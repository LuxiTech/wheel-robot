#!/usr/bin/env python3
"""Test the real console DOM with an in-page fake API and headless Chrome."""

import argparse
import re
import subprocess
import tempfile
from pathlib import Path

BOOT = r"""
const realTimeout = window.setTimeout.bind(window);
window.setTimeout = (fn, ms) => ms >= 100 ? 0 : realTimeout(fn, ms);
window.setInterval = () => 0;
document.hasFocus = () => true;
window.testPosts = [];
window.testFailures = [];
window.addEventListener('error', e => testFailures.push(e.message));
window.testModel = {
  system:{services:{arm_startup:true,control:true,episode:true,base:true,lift:true},
    free_gib:10,data_directory:'/mock',base_enabled:true,mock:true,no_cameras:true},
  arm_startup:{fresh:true,configured:true,phase:'idle',reason:'waiting_for_web_button',
    owned:false,max_error_rad:0,manual_pose:{phase:'inactive'}},
  arms:{fresh:true,enabled:false,phase:'disabled',valid:Array(16).fill(true),position:Array(16).fill(0)},
  quest:{fresh:false,rate_hz:0,left_tracking:0,right_tracking:0,grips:[0,0],sticks:[0,0,0,0]},
  control:{fresh:true,phase:'disabled',shared_enable:true,system_phase:'disabled',
    parking_policy:'legacy_timed_return_lift_stopped'},
  recorder:{fresh:true,state:'idle',samples:0,failure_menu:false},
  cameras:Object.fromEntries(['head_rgb','left_wrist_rgb','right_wrist_rgb'].map(id=>[id,{fresh:false,enabled:false}])),
  base:{valid:true,voltage:24.2,velocity:[0,0,0],flag_stop:0},
  base_status:{fresh:true,feedback_fresh:true,receive_only:false,enabled:false,
    shared_enable:true,shared_phase:'disabled',keyboard_takeover_available:true,
    web_keyboard_control:true,input_source:'quest',keyboard_owner:''},
  lift_status:{fresh:true,feedback_healthy:true,stationary:true,allow_motion:true},lift:{},server_mono_ns:0
};
window.startDeferred = null;
window.failNextKeyboard = false;
window.keyboardDeferred = null;
window.fetch = async (url, options = {}) => {
  let response = {};
  if (options.method) {
    const body = JSON.parse(options.body);
    testPosts.push({url,body});
    if (body.command === 'keyboard' && failNextKeyboard) {
      failNextKeyboard = false; throw new Error('simulated slow request');
    }
    if (body.command === 'keyboard' && keyboardDeferred) {
      const deferred = keyboardDeferred; keyboardDeferred = null;
      await new Promise(resolve => { deferred.resolve = resolve; });
    }
    if (body.command === 'keyboard_start') {
      if (startDeferred) await new Promise(resolve => { startDeferred.resolve = resolve; });
      Object.assign(testModel.base_status,{enabled:true,input_source:'keyboard',keyboard_owner:body.detail,keyboard_takeover_available:false});
    }
    if (body.command === 'stop' && (body.detail === testModel.base_status.keyboard_owner ||
        (body.detail === '__web_operator_release__' && testModel.base_status.input_source === 'keyboard')))
      Object.assign(testModel.base_status,{enabled:false,input_source:'quest',keyboard_owner:'',keyboard_takeover_available:true});
    response = {success:true,message:'accepted'};
  } else if (url === '/api/state') response = testModel;
  else if (url === '/api/devices') response = {devices:{}};
  else response = [];
  return {ok:true,status:200,text:async()=>JSON.stringify(response)};
};
"""

CHECKS = r"""
(async()=>{
 const results = [];
 const assert = (ok, message) => { if (!ok) throw new Error(message); };
 const flush = () => new Promise(resolve => realTimeout(resolve, 10));
 const switchButton = document.getElementById('keyboard-start');
 await flush();
 assert(testFailures.length === 0, 'initial script errors: ' + testFailures);
 assert(testPosts.length === 0, 'page load issued commands');
 assert(!switchButton.disabled, 'web unavailable without Quest');
 assert(switchButton.getAttribute('aria-checked') === 'false', 'switch not off by default');
 results.push('default off; available without Quest');

 switchButton.click(); await flush();
 assert(switchButton.getAttribute('aria-checked') === 'true', 'web did not acquire');
 const owner = testPosts.find(p=>p.body.command === 'keyboard_start').body.detail;
 assert(owner.startsWith('web-'), 'missing scoped owner');
 document.getElementById('keyboard-panel').dispatchEvent(new KeyboardEvent('keydown',{code:'KeyW',bubbles:true})); await flush();
 const motion = testPosts.filter(p=>p.body.command === 'keyboard').at(-1);
 assert(JSON.parse(motion.body.detail).velocity[0] === 1, 'W not mapped');
 assert(JSON.parse(motion.body.detail).owner === owner, 'unscoped motion');
 results.push('switch acquisition and WASD use scoped commands');
 if (document.getElementById('key-q')) {
   const panel = document.getElementById('keyboard-panel');
   const key = (type, code) => panel.dispatchEvent(new KeyboardEvent(type,{code,bubbles:true}));
   const velocity = () => JSON.parse(testPosts.filter(p=>p.body.command==='keyboard').at(-1).body.detail).velocity;
   key('keyup','KeyW'); await flush();
   key('keydown','KeyQ'); await flush();
   assert(JSON.stringify(velocity()) === '[0,0,1]', 'Q must request pure positive yaw');
   key('keyup','KeyQ'); await flush();
   assert(velocity().every(v=>v===0), 'Q release must stop');
   key('keydown','KeyE'); await flush();
   assert(JSON.stringify(velocity()) === '[0,0,-1]', 'E must request pure negative yaw');
   key('keyup','KeyE'); await flush();
   const q = document.getElementById('key-q'), e = document.getElementById('key-e');
   q.dispatchEvent(new PointerEvent('pointerdown',{button:0,pointerId:1,bubbles:true})); await flush();
   assert(JSON.stringify(velocity()) === '[0,0,1]' && q.getAttribute('aria-pressed')==='true', 'hold left button');
   q.dispatchEvent(new PointerEvent('pointerup',{button:0,pointerId:1,bubbles:true})); await flush();
   assert(velocity().every(v=>v===0), 'release left button');
   e.dispatchEvent(new PointerEvent('pointerdown',{button:0,pointerId:2,bubbles:true})); await flush();
   assert(JSON.stringify(velocity()) === '[0,0,-1]', 'hold right button');
   e.dispatchEvent(new PointerEvent('pointercancel',{pointerId:2,bubbles:true})); await flush();
   assert(velocity().every(v=>v===0), 'cancel right button');
   const delayed = {}; keyboardDeferred = delayed;
   key('keydown','KeyQ'); await flush();
   key('keyup','KeyQ'); await flush();
   assert(velocity().every(v=>v===0), 'release must overtake a pending turn request');
   delayed.resolve(); await flush();
   assert(keyboardPending===0 && !heldKeys.has('KeyQ'), 'late turn response must not restore held input');
   results.push('Q/E pure yaw, held page buttons, release/cancel and delayed-turn stop');
 }

 Object.assign(testModel.quest,{fresh:true,rate_hz:80}); render(testModel);
 assert(document.querySelector('[data-kind="control"][data-command="start"]').disabled,
   'Quest start allowed while web owns base');
 results.push('Quest connection does not steal web ownership');
 switchButton.click(); await flush(); render(testModel);
 assert(switchButton.getAttribute('aria-checked') === 'false', 'switch did not release');
 assert(testPosts.at(-1).body.command === 'stop' && testPosts.at(-1).body.detail === owner,
   'release not scoped');
 results.push('switch off stops and releases');

 Object.assign(testModel.base_status,{enabled:true,input_source:'quest',shared_phase:'active',keyboard_takeover_available:false});
 render(testModel); assert(switchButton.disabled, 'web allowed during Quest');
 Object.assign(testModel.base_status,{enabled:false,shared_phase:'disabled',keyboard_takeover_available:true});
 render(testModel); switchButton.click(); await flush();
 window.dispatchEvent(new Event('blur')); await flush(); render(testModel);
 assert(switchButton.getAttribute('aria-checked') === 'true', 'blur released manual ownership');
 assert(testPosts.at(-1).body.command === 'keyboard' && JSON.parse(testPosts.at(-1).body.detail).velocity.every(v=>v===0),
   'blur did not request zero velocity');
 results.push('Quest occupancy blocks web; blur stops while retaining ownership');

 document.getElementById('keyboard-panel').dispatchEvent(new KeyboardEvent('keydown',{code:'KeyW',bubbles:true}));
 await flush(); failNextKeyboard = true; await keyboardTick(); await flush();
 assert(switchButton.getAttribute('aria-checked') === 'true', 'request failure released manual ownership');
 document.getElementById('keyboard-panel').dispatchEvent(new KeyboardEvent('keydown',{code:'KeyW',repeat:true,bubbles:true}));
 await keyboardTick(); await flush();
 assert(JSON.parse(testPosts.at(-1).body.detail).velocity.every(v=>v===0), 'repeated held key replayed motion');
 const ordered = testPosts.filter(p=>p.body.command==='keyboard').map(p=>JSON.parse(p.body.detail));
 assert(ordered.every((p,i)=>p.sequence>0 && (!i || p.owner!==ordered[i-1].owner || p.sequence>ordered[i-1].sequence)), 'command sequence did not increase');
 offline(); await flush();
 assert(switchButton.getAttribute('aria-checked') === 'true', 'telemetry poll failure released manual ownership');
 render(testModel); switchButton.click(); await flush(); render(testModel);
 results.push('request and telemetry failures retain ownership; repeated held keys stay stopped');

 testModel.base_status.web_keyboard_control = false; render(testModel);
 assert(switchButton.disabled, 'old driver falsely allows web');
 testModel.base_status.web_keyboard_control = true; render(testModel);
 startDeferred = {};
 switchButton.click(); await flush();
 document.getElementById('keyboard-stop').click(); await flush();
 startDeferred.resolve(); await flush(); startDeferred = null;
 assert(switchButton.getAttribute('aria-checked') === 'false', 'late start revived manually released page');
 assert(testPosts.at(-1).body.command === 'stop' && testPosts.at(-1).body.detail,
   'late-start cleanup not scoped');
 results.push('old driver rejected; late start cannot revive manually released page');
 Object.assign(testModel.base_status,{enabled:true,input_source:'keyboard',keyboard_owner:'abandoned-owner',keyboard_takeover_available:false});
 render(testModel); assert(switchButton.disabled, 'abandoned owner was silently stolen');
 document.getElementById('keyboard-stop').click(); await flush(); render(testModel);
 assert(!testModel.base_status.enabled, 'operator release did not clear abandoned web owner');
 assert(testPosts.at(-1).body.detail==='__web_operator_release__', 'operator release was not scoped to web');
 results.push('operator can explicitly release an abandoned web page');
 assert(testFailures.length === 0, 'browser errors: ' + testFailures);
 document.body.dataset.testStatus = 'passed';
 document.body.dataset.testResults = JSON.stringify(results);
})().catch(error=>{
 document.body.dataset.testStatus='failed';
 document.body.dataset.testResults=String(error.stack || error);
});
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("app", type=Path)
    parser.add_argument("html", type=Path)
    parser.add_argument("--chrome", default="/opt/google/chrome/chrome")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    html = re.sub(r"<script\b[^>]*src=[^>]*></script>", "", args.html.read_text())
    scripts = "<script>" + BOOT + "</script><script>" + args.app.read_text() + "</script>"
    scripts += "<script>" + CHECKS + "</script>"
    html = html.replace("</body>", scripts + "</body>")
    with tempfile.TemporaryDirectory(prefix="web-base-browser-") as directory:
        page = Path(directory) / "test.html"
        page.write_text(html)
        result = subprocess.run(
            [
                args.chrome,
                "--headless=new",
                "--no-sandbox",
                "--disable-gpu",
                "--no-proxy-server",
                "--virtual-time-budget=4000",
                "--dump-dom",
                page.as_uri(),
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )
        args.output.write_text(result.stdout)
        match = re.search(r'data-test-status="([^"]*)"', result.stdout)
        if result.returncode or match is None or match.group(1) != "passed":
            details = re.search(r'data-test-results="([^"]*)"', result.stdout)
            raise SystemExit(details.group(1) if details else result.stderr[-2000:])
        print(
            "PASS: manual web ownership, zero-on-blur, transport recovery, sequences and explicit release"
        )


if __name__ == "__main__":
    main()
