"""Bounded-memory wall-clock viewport metrics; no scheduling changes."""
import time
from navigation_console_ipc import atomic_json

class RenderMetrics:
    def __init__(self, folder, profile, dt):
        self.folder=folder
        self.profile=profile
        self.dt=dt
        self.last_write=time.perf_counter()
        self.total_steps=0
        self.counts=[0,0]
        self.step_seconds=[0.,0.]
        self.wall_seconds=0.
        self.viewport_resolution=None

    def record(self, rendered, step_seconds, total_seconds):
        self.total_steps+=1
        if self.total_steps>240:
            index=int(rendered)
            self.counts[index]+=1
            self.step_seconds[index]+=step_seconds
            self.wall_seconds+=total_seconds
        if time.perf_counter()-self.last_write>=5.:
            self.write()

    def write(self):
        total=self.wall_seconds
        steps=sum(self.counts)
        report=dict(profile=self.profile,updated_at=time.time(),measured_steps=steps,
                    rendered_frames=self.counts[1],wall_seconds=total,
                    display_fps=self.counts[1]/total if total else 0.,
                    realtime_factor=steps*self.dt/total if total else 0.,
                    rendered_step_ms=1000*self.step_seconds[1]/self.counts[1] if self.counts[1] else None,
                    unrendered_step_ms=1000*self.step_seconds[0]/self.counts[0] if self.counts[0] else None,
                    viewport_resolution=self.viewport_resolution,
                    note='Rendered step includes physics; FPS counts submitted frames, not monitor presentation. Initial 240 physics steps excluded.')
        atomic_json(self.folder/'relay_render_performance.json',report)
        atomic_json(self.folder/('relay_render_'+self.profile+'.json'),report)
        self.last_write=time.perf_counter()
        return report
