"""Timestamped measured feedback and fail-closed task transitions."""
from collections import deque
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



def measured_state(state, mapping, request_seconds):
    from wheel_sdk_bridge.sdk import arm_feedback, LiftMapping
    if state['system'].get('mock') is not False:
        raise ValueError('Real SDK state required')
    arms = state['arms']
    if not isinstance(arms, dict):
        raise ValueError('SDK arms feedback unavailable: /api/state arms=null; restore measured arm feedback before planning')
    age = (int(state['server_mono_ns'])-int(arms['receive_mono_ns']))/1e9 + request_seconds
    if not 0 <= age <= .15:
        raise ValueError('Arm snapshot stale: %.3f seconds' % age)
    joints = arm_feedback(arms, max_age_ns=max(0, int((.15-age)*1e9)))
    if mapping is None:
        raise ValueError('lift_mapping missing: configure measured offset_m and direction')
    lift = state['lift']
    if not lift or lift.get('valid') is not True:
        raise ValueError('SDK lift feedback unavailable/invalid')
    age = (int(state['server_mono_ns'])-int(lift['receive_mono_ns']))/1e9 + request_seconds
    if not 0 <= age <= .15 or not 0 <= float(lift['effective_age_ms'])+age*1000 <= 150:
        raise ValueError('SDK lift feedback stale')
    joints['lift_joint'] = LiftMapping(**mapping).to_joint(lift['current_um'])
    return joints
