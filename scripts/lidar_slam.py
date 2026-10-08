"""Online 2-D scan-to-map SLAM. Inputs are ranges and encoder odometry only.

The map is built incrementally; no scene colliders or simulator poses are read.
A prior fixes the initial gauge. This compact local SLAM does not optimize a
loop-closure pose graph; confidence loss stops navigation rather than guessing.
"""
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rotation2(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class LidarSLAM:
    def __init__(self, initial_pose, resolution=.05, max_range=9, beams=240, bounds=(-12,-10,12,10)):
        self.pose = np.asarray(initial_pose, dtype=float).copy()
        self.resolution = resolution
        self.max_range = max_range
        self.angles = np.linspace(-math.pi, math.pi, beams, endpoint=False)
        self.directions = np.c_[np.cos(self.angles), np.sin(self.angles)]
        self.origin = np.array(bounds[:2], dtype=float)
        self.shape = tuple(np.ceil((np.array(bounds[2:]) - self.origin)[::-1] / resolution).astype(int))
        self.log_odds = np.zeros(self.shape, dtype=np.float32)
        self.scans = 0
        self.match_error = 0.0
        self.trajectory = []
        self.odom_pose = self.pose.copy()

    def predict(self, body_twist, dt):
        twist = np.asarray(body_twist)
        if not math.isfinite(dt) or dt <= 0 or twist.shape != (3,) or not np.all(np.isfinite(twist)):
            raise ValueError('Invalid odometry')
        for pose in (self.pose, self.odom_pose):
            pose[:2] += rotation2(pose[2] + twist[2] * dt / 2) @ twist[:2] * dt
            pose[2] = wrap(pose[2] + twist[2] * dt)

    def cells(self, xy):
        return np.floor((np.asarray(xy) - self.origin) / self.resolution).astype(int)

    def valid(self, cells):
        return ((cells[..., 0] >= 0) & (cells[..., 0] < self.shape[1]) &
                (cells[..., 1] >= 0) & (cells[..., 1] < self.shape[0]))

    def update(self, ranges):
        ranges = np.asarray(ranges, dtype=float)
        if (ranges.shape != self.angles.shape or not np.all(np.isfinite(ranges))
                or np.any(ranges < 0) or np.any(ranges > self.max_range)) :
            raise ValueError('Invalid laser scan')
        hit = (ranges > .20) & (ranges < self.max_range - .02)
        local = self.directions[hit] * ranges[hit, None]
        if self.scans > 2 and len(local) >= 20:
            distance = distance_transform_edt(self.log_odds < .7) * self.resolution
            predicted = self.pose.copy()
            # Coarse-to-fine correlative search, with a weak encoder prior.
            for xy_step, a_step in ((.035, .012), (.012, .004), (.004, .0015)):
                candidates = self.pose + np.array([[dx, dy, da] for dx in (-xy_step,0,xy_step)
                                                  for dy in (-xy_step,0,xy_step) for da in (-a_step,0,a_step)])
                costs = []
                for p in candidates:
                    points = local @ rotation2(p[2]).T + p[:2]
                    uv = (points - self.origin) / self.resolution - .5
                    d = map_coordinates(distance, [uv[:,1],uv[:,0]], order=1, mode='constant', cval=.5)
                    costs.append(np.mean(np.minimum(d, .25)**2) + .02*np.sum((p[:2]-predicted[:2])**2) + .02*wrap(p[2]-predicted[2])**2)
                self.pose = candidates[int(np.argmin(costs))]
            self.pose[2] = wrap(self.pose[2])
            points = local @ rotation2(self.pose[2]).T + self.pose[:2]
            uv = (points-self.origin)/self.resolution-.5
            errors = map_coordinates(distance,[uv[:,1],uv[:,0]],order=1,mode='constant',cval=1)
            self.match_error = float(np.mean(np.minimum(errors,.5)))
            if self.match_error > .18:
                self.pose = predicted
                return False  # Never corrupt the map with a rejected alignment.
        elif self.scans > 2:
            return False
        self.integrate(ranges, hit)
        self.scans += 1
        self.trajectory.append(self.pose.copy())
        return True

    def integrate(self, ranges, hit):
        # Vectorized beam traversal at sub-cell spacing; unique free cells per scan
        # prevent excessive confidence near the laser origin.
        distances = np.arange(.20, self.max_range, self.resolution*.8)
        directions = self.directions @ rotation2(self.pose[2]).T
        samples = self.pose[:2] + directions[:,None,:] * distances[None,:,None]
        cells = self.cells(samples)
        free = (distances[None,:] < ranges[:,None] - self.resolution) & self.valid(cells)
        free_cells = np.unique(cells[free], axis=0)
        self.log_odds[free_cells[:,1],free_cells[:,0]] -= .35
        ends = self.cells(self.pose[:2] + directions[hit] * ranges[hit,None])
        ends = ends[self.valid(ends)]
        self.log_odds[ends[:,1],ends[:,0]] += 1.0
        np.clip(self.log_odds, -5, 5, out=self.log_odds)

    def planning_grid(self, radius):
        occupied = self.log_odds > .6
        clearance = distance_transform_edt(~occupied) * self.resolution
        return clearance < radius

    def plan(self, goal, radius=.43):
        blocked = self.planning_grid(radius)
        start, end = self.cells([self.pose[:2], goal])
        if not self.valid(np.array([start,end])).all() or blocked[end[1],end[0]]:
            return []
        start, end = tuple(start), tuple(end)
        queue = [(0., start)]
        cost = {start:0.}
        parent = {}
        steps = [(dx,dy) for dx in (-1,0,1) for dy in (-1,0,1) if dx or dy]
        while queue:
            _, p = heapq.heappop(queue)
            if p == end:
                path = [p]
                while p != start:
                    p = parent[p]
                    path.append(p)
                return [self.origin+(np.array(k)+.5)*self.resolution for k in reversed(path)]
            for dx,dy in steps:
                n = (p[0]+dx,p[1]+dy)
                if not (0 <= n[0] < self.shape[1] and 0 <= n[1] < self.shape[0]) or blocked[n[1],n[0]]:
                    continue
                if dx and dy and (blocked[p[1],n[0]] or blocked[n[1],p[0]]):
                    continue
                unknown = self.log_odds[n[1],n[0]] == 0
                new = cost[p]+math.hypot(dx,dy)*(1.8 if unknown else 1)
                if new < cost.get(n,float('inf')):
                    cost[n] = new
                    parent[n] = p
                    heapq.heappush(queue,(new+math.hypot(n[0]-end[0],n[1]-end[1]),n))
        return []

    def save(self, folder):
        from pathlib import Path
        from PIL import Image
        folder = Path(folder)
        np.savez_compressed(folder/'slam_map.npz', log_odds=self.log_odds, resolution=self.resolution,
                            origin=self.origin, trajectory=np.array(self.trajectory))
        image = np.full((*self.shape,3),180,dtype=np.uint8)
        image[self.log_odds < -.2] = 245
        image[self.log_odds > .6] = 25
        cells = self.cells(np.array(self.trajectory)[:,:2]) if self.trajectory else np.empty((0,2),int)
        cells = cells[self.valid(cells)]
        image[cells[:,1],cells[:,0]] = (20,110,230)
        Image.fromarray(image[::-1]).save(folder/'slam_map.png')
