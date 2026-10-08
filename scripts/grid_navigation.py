"""Metric occupancy-grid A*: inflated footprint, no corner cutting, safe shortcuts.

Map inputs are axis-aligned obstacle bounds, independent of the simulator.
Unknown/out-of-bounds cells are not traversable.
"""
import heapq
import math


class NoPath(RuntimeError):
    pass


class GridPlanner:
    def __init__(self, bounds, resolution, obstacles, radius):
        self.bounds=tuple(bounds)  # xmin, ymin, xmax, ymax
        self.resolution=float(resolution)
        self.radius=float(radius)
        self.obstacles=[tuple(o) for o in obstacles]
        if resolution<=0 or radius<0:
            raise ValueError('Invalid grid or footprint')
        self.nx=int((bounds[2]-bounds[0])/resolution)+1
        self.ny=int((bounds[3]-bounds[1])/resolution)+1
        self._blocked={}

    def cell(self,p):
        return (round((p[0]-self.bounds[0])/self.resolution),
                round((p[1]-self.bounds[1])/self.resolution))

    def point(self,c):
        return (self.bounds[0]+c[0]*self.resolution,self.bounds[1]+c[1]*self.resolution)

    def free_point(self,p,extra=0.):
        x,y=p
        r=self.radius+extra
        a,b,c,d=self.bounds
        if not (a+r<=x<=c-r and b+r<=y<=d-r):
            return False
        for x0,y0,x1,y1 in self.obstacles:
            dx=max(x0-x,0.,x-x1)
            dy=max(y0-y,0.,y-y1)
            if dx*dx+dy*dy<=r*r:
                return False
        return True

    def free(self,c):
        if c not in self._blocked:
            self._blocked[c]=not (0<=c[0]<self.nx and 0<=c[1]<self.ny and
                self.free_point(self.point(c),self.resolution/math.sqrt(2)))
        return not self._blocked[c]

    def visible(self,a,b):
        n=max(1,math.ceil(math.dist(a,b)/(self.resolution*.4)))
        return all(self.free_point((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n),
                                   self.resolution*.25) for i in range(n+1))

    def plan(self,start,goal):
        start,goal=tuple(start),tuple(goal)
        s,g=self.cell(start),self.cell(goal)
        if not self.free(s) or not self.free(g) or not self.free_point(start) or not self.free_point(goal):
            raise NoPath('Start or goal is inside an inflated obstacle or outside the map')
        queue=[(math.dist(s,g),0.,s)]
        costs={s:0.}
        parent={}
        while queue:
            _,cost,c=heapq.heappop(queue)
            if cost>costs[c]:
                continue
            if c==g:
                cells=[g]
                while cells[-1]!=s:
                    cells.append(parent[cells[-1]])
                points=[start]+[self.point(c) for c in reversed(cells)][1:-1]+[goal]
                # Preserve exact endpoints only if their connections are safe.
                if any(not self.visible(a,b) for a,b in zip(points,points[1:])):
                    raise NoPath('Exact endpoint connection is blocked')
                result=[points[0]]
                i=0
                while i<len(points)-1:
                    j=len(points)-1
                    while j>i+1 and not self.visible(points[i],points[j]):
                        j-=1
                    result.append(points[j]);i=j
                return result
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=(c[0]+dx,c[1]+dy)
                if not self.free(nxt):
                    continue
                if dx and dy and (not self.free((c[0]+dx,c[1])) or not self.free((c[0],c[1]+dy))):
                    continue
                new=cost+math.hypot(dx,dy)
                if new<costs.get(nxt,float('inf')):
                    costs[nxt]=new;parent[nxt]=c
                    heapq.heappush(queue,(new+math.dist(nxt,g),new,nxt))
        raise NoPath('No collision-free path')


def collider_bounds(colliders,low=.02,high=1.6):
    result=[]
    for item in colliders:
        x,y,z=item['position'];sx,sy,sz=item['size']
        if item['name']=='Floor' or z+sz/2<low or z-sz/2>high:
            continue
        result.append((x-sx/2,y-sy/2,x+sx/2,y+sy/2))
    return result


def path_length(points):
    return sum(math.dist(a,b) for a,b in zip(points,points[1:]))
