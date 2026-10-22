"""Real feedback mappings and local leases. This module sends no commands."""
from dataclasses import dataclass
import math
import time
import uuid


@dataclass
class LiftMapping:
    direction: int
    offset_m: float
    device_min_um: int
    device_max_um: int
    margin_um: int=5000
    urdf_min: float=0
    urdf_max: float=.4

    def __post_init__(self):
        if self.direction not in (-1,1) or not math.isfinite(self.offset_m) or self.margin_um<0:
            raise ValueError('Unvalidated lift mapping')
        if self.device_min_um+self.margin_um>self.device_max_um-self.margin_um:
            raise ValueError('Empty device travel')

    def to_joint(self,current_um):
        value=self.direction*float(current_um)*1e-6+self.offset_m
        if not math.isfinite(value) or not self.urdf_min<=value<=self.urdf_max:
            raise ValueError('Lift feedback outside URDF range')
        return value

    def to_device(self,q):
        if not math.isfinite(q) or not self.urdf_min<=q<=self.urdf_max:
            raise ValueError('Lift target outside URDF range')
        v=round((q-self.offset_m)/self.direction*1e6)
        if not self.device_min_um+self.margin_um<=v<=self.device_max_um-self.margin_um:
            raise ValueError('Lift target outside inset device range')
        return v


def arm_feedback(state,*,max_age_ns=150_000_000):
    for key in ('position','valid','age_ns'):
        if len(state[key])!=16: raise ValueError('Expected 16 SDK joints')
    if state.get('fresh') is False: raise ValueError('Stale arm feedback')
    values={}
    for side,start in [('right',0),('left',8)]:
        for j in range(7):
            idx=start+j
            if (state['valid'][idx] is not True or not 0<=int(state['age_ns'][idx])<=max_age_ns
                    or not math.isfinite(state['position'][idx])):
                raise ValueError('Invalid/stale arm joint '+str(idx))
            values[f'openarm_{side}_joint{j+1}']=float(state['position'][idx])
    return values


class LocalLease:
    """Construct SDK lease packets only on robot. Independent remote timeout.

    Caller must supply a monotonic clock from the robot process. No transport
    is provided: using this class on a workstation is a dry-run only.
    """
    def __init__(self,clock=time.monotonic_ns,remote_timeout_ns=150_000_000):
        self.clock=clock;self.remote_timeout=remote_timeout_ns
        self.owner=None;self.sequence=0;self.revoked=True;self.remote_seq=0
        self.remote_time=None;self.last_local=None;self.last_task=0;self.stream=None;self.armed_remote_seq=0

    def remote_input(self,sequence):
        if type(sequence)!=int or sequence<=self.remote_seq: raise ValueError('Stale remote sequence')
        self.remote_seq=sequence;self.remote_time=self.clock()

    def begin(self,stream):
        if stream not in ('base','lift'): raise ValueError('Unsupported SDK stream')
        if not self.revoked: raise ValueError('Lease already active')
        self._fresh()
        if self.remote_seq<=self.armed_remote_seq: raise ValueError('New explicit remote arm required')
        self.armed_remote_seq=self.remote_seq
        self.owner=str(uuid.uuid4());self.sequence=0;self.last_task=0;self.stream=stream
        self.revoked=False;self.last_local=self.clock()
        return self._packet('arm')

    def _fresh(self):
        now=self.clock()
        if self.remote_time is None or not 0<=now-self.remote_time<=self.remote_timeout:
            self.revoked=True;raise ValueError('Remote control input expired')

    def command(self,op,**payload):
        if self.revoked: raise ValueError('Explicit new session required')
        now=self.clock()
        keys={'velocity':{'velocity'},'move':{'task_id','target_um'},'heartbeat':set(),'stop':set()}
        if op not in keys or set(payload)!=keys[op]:
            self.revoked=True;raise ValueError('Unexpected command payload')
        if op=='stop':
            packet=self._packet(op);self.revoked=True;return packet
        self._fresh()
        if not 0<=now-self.last_local<=200_000_000:
            self.revoked=True;raise ValueError('Local lease expired')
        allowed={'base':{'velocity'},'lift':{'heartbeat','move'}}[self.stream]
        if op not in allowed:
            self.revoked=True;raise ValueError('Invalid operation for stream')
        if op=='velocity':
            v=payload.get('velocity')
            if (not isinstance(v,(list,tuple)) or len(v)!=3 or not all(math.isfinite(x) for x in v)
                    or math.hypot(*v[:2])>.10 or abs(v[2])>.20):
                self.revoked=True;raise ValueError('Invalid base velocity')
        if op=='move':
            task=payload.get('task_id');target=payload.get('target_um')
            if type(task)!=int or task<=self.last_task or type(target)!=int:
                self.revoked=True;raise ValueError('Invalid lift task')
            self.last_task=task
        return self._packet(op,**payload)

    def _packet(self,op,**payload):
        self.sequence+=1;self.last_local=self.clock()
        return dict(owner=self.owner,sequence=self.sequence,source_mono_ns=str(self.last_local),op=op,**payload)
