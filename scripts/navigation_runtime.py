"""Fixed pickup stations reached with A*, independently of manipulation skills."""
import json
import math
import time
import numpy as np
from common import ROOT,OUTPUTS
from relay_runtime import RelayRuntime,wrap
from relay_task import RelayFault
from grid_navigation import GridPlanner,collider_bounds,path_length


class NavigationRuntime(RelayRuntime):
    def __init__(self,*args,localization='ground_truth',goal=None,**kwargs):
        self.nav_config=json.loads((ROOT/'config/navigation_demo_config.json').read_text())
        self.nav_localization=localization
        self.estimator_configuration={name:(ROOT/'config'/name).read_text()
                                      for name in ('fast_livo_sim.yaml','fast_livo_camera.yaml')}
        self.selected_goals=[goal] if goal else [p['id'] for p in self.nav_config['points']]
        self.nav_paths=[]
        self.nav_reached=[]
        self.nav_trace=[]
        self.static_bounds=collider_bounds(json.loads((ROOT/'assets/generated/relay/static_colliders.json').read_text()))
        self.sensor_bridge=None
        super().__init__(*args,**kwargs)

    def reset(self):
        super().reset()
        self.nav_paths=[];self.nav_reached=[];self.nav_trace=[]

    def start(self):
        if self.active or self.finished:
            return
        self.active=True
        self.workers={'A':self.run_a(),'B':self.run_b()}
        self.event('A','START',localization=self.nav_localization,planner='A*')
        self.event('B','STOW')

    def select_goal(self,goal):
        if self.active:
            return
        self.reset()
        self.selected_goals=[goal] if goal else [p['id'] for p in self.nav_config['points']]
        self.start()

    def navigation_pose(self):
        if self.nav_localization=='ground_truth':
            b=self.base('A')
            return np.array([b[0],b[1],b[3]])
        if self.sensor_bridge is None:
            raise RelayFault('FAST-LIVO2 sensor bridge is not initialized')
        return self.sensor_bridge.estimated_pose(self.sensor_bridge.sensor_time)

    def planner(self,*,tracking=False):
        obstacles=list(self.static_bounds)
        # Other robot is a live obstacle. Its pose is supplied by the simulator
        # map adapter in known-map mode; it is never a localization observation.
        p=self.base('B')
        r=self.nav_config['robot_radius']
        obstacles.append((p[0]-r,p[1]-r,p[0]+r,p[1]+r))
        return GridPlanner(self.nav_config['bounds'],self.nav_config['resolution'],obstacles,
                           r if tracking else r+.06)

    def draw_path(self,path):
        from pxr import UsdGeom,Gf
        stage=next(iter(self.controls['Robot_A'].drives.values())).GetPrim().GetStage()
        curve=UsdGeom.BasisCurves.Define(stage,'/World/Navigation/PlannedPath')
        curve.CreateTypeAttr('linear')
        curve.CreateCurveVertexCountsAttr([len(path)])
        curve.CreatePointsAttr([Gf.Vec3f(float(x),float(y),.025) for x,y in path])
        curve.CreateWidthsAttr([.025])
        curve.SetWidthsInterpolation('constant')
        curve.CreateDisplayColorAttr([Gf.Vec3f(.1,1.,.25)])
        for point in self.nav_config['points']:
            sphere=UsdGeom.Sphere.Define(stage,'/World/Navigation/'+point['id'])
            sphere.CreateRadiusAttr(.065)
            sphere.CreateDisplayColorAttr([Gf.Vec3f(1.,.65,.05)])
            xf=UsdGeom.Xformable(sphere)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*point['pose'][:2],.04))

    def run_b(self):
        yield from self.stow('B')
        self.event('B','PARKED')

    def run_a(self):
        yield from self.stow('A')
        if self.nav_localization=='fast_livo2':
            self.event('A','WAIT_LOCALIZATION')
            deadline=self.time+30.
            while True:
                try:
                    self.navigation_pose()
                    break
                except RelayFault:
                    if self.time>deadline:
                        raise RelayFault('FAST-LIVO2 did not produce fresh initialized odometry')
                    yield
        for name in self.selected_goals:
            station=next(p for p in self.nav_config['points'] if p['id']==name)
            yield from self.go_to(station)
            yield from self.wait(.5)
        self.event('A','NAVIGATION_COMPLETE')

    def go_to(self,station):
        ctl=self.controls['Robot_A']
        pose=self.navigation_pose()
        goal=np.array(station['pose'])
        planner=self.planner()
        path=planner.plan(pose[:2],goal[:2])
        self.draw_path(path)
        row=dict(goal=station['id'],start=pose.tolist(),target=goal.tolist(),
                 path=path,length_m=path_length(path),planner='A*',map_source='known_scene_static_colliders')
        self.nav_paths.append(row)
        self.event('A','ASTAR_NAVIGATE',goal=station['id'],length_m=row['length_m'],waypoints=len(path))
        start=self.time;last_progress=start;best=float('inf');idx=1;stable=0.
        travel_yaw=float(pose[2]);aligning=False
        while self.time-start<120.:
            pose=self.navigation_pose()
            if self.sensor_bridge is not None and self.sensor_bridge.closest_obstacle<.48:
                ctl.stop()
                raise RelayFault('Raw LiDAR proximity stop: obstacle within the navigation safety radius')
            if not np.isfinite(pose).all():
                raise RelayFault('Invalid localization')
            remaining=float(np.linalg.norm(goal[:2]-pose[:2]))
            point=np.array(path[idx])
            distance=float(np.linalg.norm(point-pose[:2]))
            if idx<len(path)-1 and distance<.05 and self.planner(tracking=True).visible(pose[:2],path[idx+1]):
                idx+=1;point=np.array(path[idx]);distance=float(np.linalg.norm(point-pose[:2]))
                best=float('inf');last_progress=self.time
            if idx==len(path)-1 and remaining<.025 and not aligning:
                aligning=True;best=float('inf');last_progress=self.time
            # Omni wheels can follow XY without a half-turn during translation.
            # Align the grasp heading after reaching the station instead.
            heading=wrap((goal[2] if aligning else travel_yaw)-pose[2])
            progress=distance+.15*abs(heading)
            if progress<best-.003:
                best=progress;last_progress=self.time
            elif self.time-last_progress>8.:
                raise RelayFault('A* tracking stalled')
            if not self.planner(tracking=True).visible(pose[:2],point):
                ctl.stop()
                raise RelayFault('Planned segment became occupied; stop before replanning')
            delta=point-pose[:2]
            speed=min(self.nav_config['speed'],max(.03,distance*1.5))
            v=delta/max(distance,1e-9)*speed if distance>.015 else np.zeros(2)
            c,s=math.cos(pose[2]),math.sin(pose[2])
            body=np.array([[c,s],[-s,c]]) @ v
            turn=float(np.clip(heading*1.5,-.5,.5))
            if abs(heading)>.015 and abs(turn)<.08:
                turn=math.copysign(.08,heading)
            ctl.set_twist(*body,turn if abs(heading)>.015 else 0.)
            stable=stable+.05 if aligning and remaining<self.nav_config['goal_tolerance'] and abs(heading)<.025 else 0.
            actual=self.base('A')
            self.nav_trace.append(dict(t=self.time,pose=pose.tolist(),goal=station['id'],
                                      evaluation_pose=[float(actual[0]),float(actual[1]),float(actual[3])]))
            if stable>=.3:
                ctl.stop()
                actual=self.base('A')
                evaluation_error=float(np.linalg.norm(actual[:2]-goal[:2]))
                evaluation_angle=abs(wrap(float(actual[3]-goal[2])))
                self.nav_reached.append(dict(id=station['id'],t=self.time,position_error_m=remaining,
                                            angle_error_rad=abs(heading),evaluation_position_error_m=evaluation_error,
                                            evaluation_angle_error_rad=evaluation_angle))
                if evaluation_error>.10:
                    raise RelayFault('Navigation evaluation: estimated arrival differs from physical goal by over 10 cm')
                if evaluation_angle>.10:
                    raise RelayFault('Navigation evaluation: physical heading differs from goal by over 0.10 rad')
                self.event('A','GOAL_REACHED',goal=station['id'],position_error_m=remaining)
                return
            yield
        raise RelayFault('A* navigation timeout')

    def write_report(self):
        report=dict(passed=bool(not self.error and self.nav_reached and len(self.nav_reached)==len(self.selected_goals)),
                    error=self.error,localization=self.nav_localization,planner='A*',
                    map_source='known_scene_static_colliders',manipulation_executed=False,
                    simulation_seconds=self.time,wall_seconds=time.perf_counter()-self.wall_started,
                    goals=self.selected_goals,reached=self.nav_reached,plans=self.nav_paths,trace=self.nav_trace,
                    events=self.events,config=self.nav_config)
        if self.sensor_bridge is not None:
            report['raw_sensor_frames']=self.sensor_bridge.sequence
            report['sensor_run_id']=self.sensor_bridge.run_id
            report['imu_source']='Isaac native IMUSensor'
            report['estimator_configuration']=self.estimator_configuration
            odometry_path=OUTPUTS/'fast_livo/odometry.json'
            if odometry_path.exists():
                odometry=json.loads(odometry_path.read_text())
                if odometry.get('run_id')==self.sensor_bridge.run_id:
                    report['estimator_pipeline']=odometry.get('pipeline','unknown')
            status_path=OUTPUTS/'fast_livo/sensor_status.json'
            if status_path.exists():
                status=json.loads(status_path.read_text())
                if status['run_id']==self.sensor_bridge.run_id:
                    report['sensor_status']=status
        (OUTPUTS/'navigation_report.json').write_text(json.dumps(report,indent=2))
