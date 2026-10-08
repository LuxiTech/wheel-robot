"""CUDA batched bounded IK. FK, finite-difference Jacobians and LM run on GPU."""
import glob
import sys
from collections import OrderedDict
import numpy as np
from scipy.spatial.transform import Rotation
try:
    import warp as wp
except ImportError:
    paths=glob.glob('/isaac-sim/extscache/omni.warp.core-*/')
    if not paths:
        raise RuntimeError('CUDA IK requires NVIDIA Warp; CPU fallback is disabled')
    sys.path.insert(0,paths[0])
    import warp as wp

wp.set_module_options({"enable_backward": False})

V18=wp.types.vector(length=18,dtype=wp.float32)
V12=wp.types.vector(length=12,dtype=wp.float32)
M18=wp.types.matrix(shape=(18,18),dtype=wp.float32)
J12=wp.types.matrix(shape=(12,18),dtype=wp.float32)

@wp.func
def residual(x:V18, origins:wp.array2d(dtype=wp.transform), axes:wp.array2d(dtype=wp.vec3),
             indices:wp.array2d(dtype=wp.int32), kinds:wp.array2d(dtype=wp.int32),
             tcp:wp.array(dtype=wp.vec3), targets:wp.array(dtype=wp.transform),
             active:wp.array(dtype=wp.int32), base_z:float):
    result=V12(0.0)
    root=wp.transform(wp.vec3(x[0],x[1],base_z),wp.quat_from_axis_angle(wp.vec3(0.0,0.0,1.0),x[2]))
    for side in range(2):
        if active[side]!=0:
            t=root
            for j in range(origins.shape[1]):
                t=wp.transform_multiply(t,origins[side,j])
                idx=indices[side,j]
                if idx>=0:
                    motion=wp.transform_identity()
                    if kinds[side,j]==1:
                        motion=wp.transform(axes[side,j]*x[idx],wp.quat_identity())
                    else:
                        motion=wp.transform(wp.vec3(0.0),wp.quat_from_axis_angle(axes[side,j],x[idx]))
                    t=wp.transform_multiply(t,motion)
            p=wp.transform_point(t,tcp[side])-wp.transform_get_translation(targets[side])
            dq=wp.transform_get_rotation(targets[side])*wp.quat_inverse(wp.transform_get_rotation(t))
            if dq[3]<0.0:
                dq=-dq
            v=wp.vec3(dq[0],dq[1],dq[2])
            norm=wp.length(v)
            rv=v*2.0
            if norm>1.0e-7:
                rv=v*(2.0*wp.atan2(norm,dq[3])/norm)
            for k in range(3):
                result[side*6+k]=p[k]
                result[side*6+3+k]=0.2*rv[k]
    return result

@wp.func
def cost(r:V12,x:V18,seed:V18):
    val=float(0.0)
    for i in range(12):
        val+=r[i]*r[i]
    for i in range(18):
        val+=0.000004*(x[i]-seed[i])*(x[i]-seed[i])
    return val

@wp.kernel
def solve_kernel(starts:wp.array(dtype=V18), reference:wp.array(dtype=V18), lower:wp.array(dtype=V18),upper:wp.array(dtype=V18),
                 origins:wp.array2d(dtype=wp.transform),axes:wp.array2d(dtype=wp.vec3),
                 indices:wp.array2d(dtype=wp.int32),kinds:wp.array2d(dtype=wp.int32),tcp:wp.array(dtype=wp.vec3),
                 targets:wp.array(dtype=wp.transform),active:wp.array(dtype=wp.int32),base_z:float,
                 iterations:int,output:wp.array(dtype=V18),scores:wp.array(dtype=wp.vec3)):
    tid=wp.tid()
    x=starts[tid]
    seed=reference[0]
    lo=lower[0]
    hi=upper[0]
    damping=float(0.001)
    r=residual(x,origins,axes,indices,kinds,tcp,targets,active,base_z)
    value=cost(r,x,seed)
    for iteration in range(iterations):
        jac=J12(0.0)
        for j in range(18):
            if hi[j]-lo[j]>1.0e-7:
                xp=x
                xp[j]+=0.001
                rp=residual(xp,origins,axes,indices,kinds,tcp,targets,active,base_z)
                for i in range(12):
                    jac[i,j]=(rp[i]-r[i])/0.001
        blocked=V18(0.0)
        for j in range(18):
            gradient=0.000004*(x[j]-seed[j])
            for k in range(12):
                gradient+=jac[k,j]*r[k]
            if ((x[j]<=lo[j]+0.000001 and gradient>0.0) or
                    (x[j]>=hi[j]-0.000001 and gradient<0.0)):
                blocked[j]=1.0
                for k in range(12):
                    jac[k,j]=0.0
        h=M18(0.0)
        rhs=V18(0.0)
        for j in range(18):
            if hi[j]-lo[j]>1.0e-7 and blocked[j]==0.0:
                rhs[j]=-0.000004*(x[j]-seed[j])
                for k in range(12):
                    rhs[j]-=jac[k,j]*r[k]
            for k in range(j+1):
                v=float(0.0)
                for row in range(12):
                    v+=jac[row,j]*jac[row,k]
                if j==k:
                    v+=damping+0.000004
                h[j,k]=v
                h[k,j]=v
        # Cholesky and triangular solves, independently for each GPU seed.
        chol=M18(0.0)
        for i in range(18):
            for j in range(i+1):
                v=h[i,j]
                for k in range(j):
                    v-=chol[i,k]*chol[j,k]
                if i==j:
                    chol[i,j]=wp.sqrt(wp.max(v,1.0e-12))
                else:
                    chol[i,j]=v/chol[j,j]
        y=V18(0.0)
        for i in range(18):
            v=rhs[i]
            for j in range(i):
                v-=chol[i,j]*y[j]
            y[i]=v/chol[i,i]
        delta=V18(0.0)
        for ii in range(18):
            i=17-ii
            v=y[i]
            for j in range(i+1,18):
                v-=chol[j,i]*delta[j]
            delta[i]=v/chol[i,i]
        largest=float(0.0)
        for j in range(18):
            largest=wp.max(largest,wp.abs(delta[j]))
        scale=wp.min(1.0,0.25/wp.max(largest,1.0e-9))
        trial=V18(0.0)
        for j in range(18):
            trial[j]=wp.clamp(x[j]+scale*delta[j],lo[j],hi[j])
        rr=residual(trial,origins,axes,indices,kinds,tcp,targets,active,base_z)
        candidate=cost(rr,trial,seed)
        if candidate<value:
            x=trial
            r=rr
            value=candidate
            damping=wp.max(damping*0.5,0.000001)
        else:
            damping=wp.min(damping*4.0,100.0)
        if value<1.0e-12 or damping>=100.0:
            break
    pe=float(0.0)
    ae=float(0.0)
    for s in range(2):
        pe=wp.max(pe,wp.length(wp.vec3(r[s*6],r[s*6+1],r[s*6+2])))
        ae=wp.max(ae,5.0*wp.length(wp.vec3(r[s*6+3],r[s*6+4],r[s*6+5])))
    output[tid]=x
    scores[tid]=wp.vec3(pe,ae,value)

class CudaIK:
    def __init__(self,arms,device='cuda:0'):
        wp.init()
        self.device=wp.get_device(device)
        if not self.device.is_cuda:
            raise RuntimeError('IK device must be CUDA')
        chain_length=max(len(a.chain) for a in arms.values())
        origins=np.zeros((2,chain_length,7),dtype=np.float32);origins[:,:,6]=1
        axes=np.zeros((2,chain_length,3),dtype=np.float32)
        indices=np.full((2,chain_length),-1,dtype=np.int32)
        kinds=np.zeros((2,chain_length),dtype=np.int32)
        for s,arm in enumerate(arms.values()):
            for j,(name,kind,origin,axis) in enumerate(arm.chain):
                origins[s,j,:3]=origin[:3,3]
                origins[s,j,3:]=Rotation.from_matrix(origin[:3,:3]).as_quat()
                axes[s,j]=axis
                if name=='lift_joint': indices[s,j]=3;kinds[s,j]=1
                elif name in arm.names: indices[s,j]=4+7*s+arm.names.index(name);kinds[s,j]=2
        self.origins=wp.array(origins,dtype=wp.transform,device=self.device)
        self.axes=wp.array(axes,dtype=wp.vec3,device=self.device)
        self.indices=wp.array(indices,dtype=wp.int32,device=self.device)
        self.kinds=wp.array(kinds,dtype=wp.int32,device=self.device)
        self.tcp=wp.array(np.array([a.tcp for a in arms.values()]),dtype=wp.vec3,device=self.device)
        self.calls=0
        self.cache=OrderedDict()
        self.cache_capacity=256
        self.cache_hits=0

    def solve(self,targets,base,x0,low,high,attempts):
        key=(np.asarray(base,dtype=np.float64).tobytes(),x0.tobytes(),low.tobytes(),high.tobytes(),attempts,
             tuple((s,np.asarray(t,dtype=np.float64).tobytes()) for s,t in sorted(targets.items())))
        if self.cache_capacity and key in self.cache:
            self.cache.move_to_end(key)
            self.cache_hits+=1
            return self.cache[key].copy()
        count=max(8,attempts*8)
        rng=np.random.default_rng(42)
        starts=np.tile(x0,(count,1))
        for i in range(1,count):
            if i<count//2:
                starts[i]=np.clip(x0+rng.normal(0,.15,18),low,high)
            else:
                starts[i]=rng.uniform(low,high)
        target=np.zeros((2,7),dtype=np.float32);target[:,6]=1
        active=np.zeros(2,dtype=np.int32)
        for s,side in enumerate(('left','right')):
            if side in targets:
                active[s]=1;target[s,:3]=targets[side][:3,3]
                target[s,3:]=Rotation.from_matrix(targets[side][:3,:3]).as_quat()
        def arr(x,dtype):return wp.array(np.asarray(x),dtype=dtype,device=self.device)
        out=wp.empty(count,dtype=V18,device=self.device)
        scores=wp.empty(count,dtype=wp.vec3,device=self.device)
        wp.launch(solve_kernel,dim=count,inputs=[arr(starts,V18),arr([x0],V18),arr([low],V18),arr([high],V18),
            self.origins,self.axes,self.indices,self.kinds,self.tcp,arr(target,wp.transform),arr(active,wp.int32),
            float(base[2]),120,out,scores],device=self.device)
        values=scores.numpy();solutions=out.numpy();self.calls+=1
        good=(values[:,0]<.008)&(values[:,1]<.08)&np.isfinite(values).all(axis=1)
        ranking=values[:,2] if good.any() else values[:,0]+.2*values[:,1]
        if good.any():ranking=np.where(good,ranking,np.inf)
        index=int(np.argmin(ranking))
        answer=solutions[index].astype(float)
        if self.cache_capacity and good[index]:
            self.cache[key]=answer.copy()
            while len(self.cache)>self.cache_capacity:
                self.cache.popitem(last=False)
        return answer
