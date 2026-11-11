#!/usr/bin/env python3
"""Extend the robot's existing isolated ROS regression with enforced scoped commands."""

import os
from pathlib import Path

from prepare_compatibility import replace_once

source = "\n".join(
    line for line in Path(os.environ["SDK_LEGACY_TEST"]).read_text().splitlines() if line.strip()
)
source = replace_once(
    source, "os.environ['ROS_DOMAIN_ID']='177'", "os.environ['ROS_DOMAIN_ID']='178'"
)
source = replace_once(
    source,
    "        events=[]",
    """        events=[]
        sdk_owner=''
        sdk_sequence=0
        control_source='console'""",
)
source = replace_once(
    source,
    "            nonlocal enabled",
    """            nonlocal enabled, sdk_owner, sdk_sequence
            if request.command=='keyboard_start':
                assert request.detail.startswith('atlas:'), request.detail
                sdk_owner=request.detail
                sdk_sequence=0
            elif request.command=='keyboard':
                value=json.loads(request.detail)
                assert value['owner']==sdk_owner
                assert value['sequence']>sdk_sequence
                sdk_sequence=value['sequence']
                assert all(abs(speed)<=1 for speed in value['velocity'])
            elif request.command=='stop':
                assert request.detail==sdk_owner
                sdk_owner=''""",
)
source = replace_once(
    source,
    "dict(enabled=enabled,receive_only=False,feedback_fresh=True)",
    "dict(enabled=enabled,receive_only=False,feedback_fresh=True,shared_enable=True,web_keyboard_control=True,web_keyboard_linear_limit=.05,control_source=control_source,keyboard_takeover_available=not enabled)",
)
source = replace_once(source, "command_seq=seq,x=1,y=0", "command_seq=seq,x=.1,y=0")
source = replace_once(
    source,
    "json.loads(detail)==[1,0,0]",
    "json.loads(detail)['velocity']==[1,0,0]",
)
source = replace_once(
    source,
    "self.assertEqual(json.loads(commands[-1][2]),[0,0,0])",
    "self.assertEqual(json.loads(commands[-1][2])['velocity'],[0,0,0])",
)
source = replace_once(
    source,
    "            feedback=False;time.sleep(.35)",
    """            self.assertEqual(request('/api/relay/status')[1]['linear_limit'],.05)
            control_source='teleop';time.sleep(.15)
            count=len(events)
            self.assertEqual(request('/api/keyboard',dict(command='keyboard_start'))[0],503)
            self.assertEqual(len(events),count)
            control_source='console'
            feedback=False;time.sleep(.35)""",
)
exec(compile(source, "isolated_scoped_sdk_ros_regression", "exec"))
