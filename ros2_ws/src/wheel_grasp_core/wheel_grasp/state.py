"""Timestamped measured feedback and fail-closed task transitions."""
from collections import deque
from dataclasses import dataclass
import numpy as np


class JointHistory:
    def __init__(self,capacity=256,max_gap=.1):
        self.samples=deque(maxlen=capacity);self.max_gap=max_gap

    def append(self,stamp,joints):
        if not np.isfinite(stamp) or not joints or not all(np.isfinite(v) for v in joints.values()):
            raise ValueError('Invalid feedback sample')
        if self.samples and stamp<=self.samples[-1][0]: raise ValueError('Out-of-order feedback')
        self.samples.append((stamp,dict(joints)))

    def at(self,stamp):
        for t,q in self.samples:
            if t==stamp: return q.copy()
        for (a,qa),(b,qb) in zip(self.samples,list(self.samples)[1:]):
            if a<stamp<b:
                if b-a>self.max_gap or qa.keys()!=qb.keys(): raise ValueError('Feedback gap or incomplete joints')
                u=(stamp-a)/(b-a)
                return {n:qa[n]+u*(qb[n]-qa[n]) for n in qa}
        raise ValueError('No feedback bracketing observation; extrapolation forbidden')


@dataclass
class TaskState:
    phase: str='IDLE'
    target_id: str=''
    pair_id: str=''
    holding: bool=False
    reason: str=''
    pregrasp_completed: bool=False

    def begin(self,target_id):
        if self.phase!='IDLE' or not target_id: raise ValueError('Task cannot start')
        self.target_id=target_id;self.phase='ACQUIRE'

    def observed(self,target_id,pair_id,*,quality_valid):
        if self.phase not in ('ACQUIRE','REOBSERVE') or target_id!=self.target_id or not pair_id:
            raise ValueError('Observation does not belong to task')
        if not quality_valid:
            self.phase='ACQUIRE';self.reason='quality_rejected';return
        self.pair_id=pair_id;self.phase='PLAN';self.reason=''

    def planned(self,pair_id,*,collision_checked,hardware_ready):
        if self.phase!='PLAN' or pair_id!=self.pair_id: raise ValueError('Stale plan')
        if not collision_checked or not hardware_ready:
            self.reason='execution_not_ready';return
        self.phase='PREGRASP'

    def advance(self,event):
        transitions={('PREGRASP','arrived'):'REOBSERVE',('APPROACH','arrived'):'CLOSE',
                     ('CLOSE','contact_verified'):'TEST_LIFT',('TEST_LIFT','hold_verified'):'CARRY',
                     ('CARRY','arrived'):'PLACE',('PLACE','supported_release'):'COMPLETE'}
        if (self.phase,event) not in transitions: raise ValueError('Invalid task transition')
        if self.phase=='PREGRASP' and event=='arrived': self.pregrasp_completed=True
        self.phase=transitions[(self.phase,event)]
        if event=='contact_verified':self.holding=True
        if event=='supported_release':self.holding=False

    def final_approach(self,pair_id,*,verified):
        # After reobservation the planner must explicitly accept the updated plan.
        if self.phase!='PLAN' or pair_id!=self.pair_id or not verified or not self.pregrasp_completed: raise ValueError('Approach not verified')
        self.phase='APPROACH'

    def cancel(self,reason):
        self.phase='HOLD' if self.holding else 'CANCELLED';self.reason=reason
        # State only: no implicit release, return-home or motor disable.
