#!/usr/bin/env python3
"""Verify the console source selector with real DOM and an in-page mock API."""

import argparse
import importlib.util
import re
import subprocess
import tempfile
from pathlib import Path

fixture_path = Path(__file__).resolve().parents[1] / "base_console/test_console_browser.py"
spec = importlib.util.spec_from_file_location("base_browser", fixture_path)
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)

MODEL = """
testModel.control.control_source = 'console';
testModel.control.vr_fresh = true;
testModel.control.quest_window_samples = 38;
testModel.arm_startup.joint_control = {phase:'idle',unit:'rad',joint_order:'right_then_left',lower:Array(16).fill(-3),upper:Array(16).fill(3)};
Object.assign(testModel.lift_status,{dashboard_position_available:true,enabled:false});
const oldFetch = window.fetch;
window.fetch = async (url, options={}) => {
  if (options.method && url==='/api/control') {
    const body=JSON.parse(options.body);
    if (body.command.startsWith('select_')) testModel.control.control_source=body.command.slice(7);
  }
  return oldFetch(url,options);
};
"""

CHECKS = """
(async()=>{
 const assert=(ok,message)=>{if(!ok)throw new Error(message);};
 const flush=()=>new Promise(resolve=>realTimeout(resolve,20));
 await flush();
 assert(testFailures.length===0,'script errors: '+testFailures);
 assert(testPosts.length===0,'page load submitted commands');
 const consoleButton=$('source-console'),teleopButton=$('source-teleop');
 assert(consoleButton.getAttribute('aria-pressed')==='true','console not selected');
 assert(!teleopButton.disabled,'explicit switch unavailable');
 Object.assign(testModel.quest,{fresh:true,rate_hz:80,sticks:[0,1,0,0],grips:[.8,.8]});
 render(testModel); await flush();
 $('joint-read-current').click(); await flush();
 assert(!$('joint-execute').disabled,'connected Quest blocked joint controls');
 assert(!$('keyboard-start').disabled,'connected Quest blocked base controls');
 assert(!$('lift-position-enable').disabled,'connected Quest blocked lift controls');
 assert(testPosts.length===0,'Quest connection sent commands');
 teleopButton.click(); await flush();
 assert(teleopButton.getAttribute('aria-pressed')==='true','teleop selection missing');
 assert($('joint-execute').disabled && $('keyboard-start').disabled && $('lift-position-enable').disabled,'console actuator controls bypassed selection');
 assert(testPosts.length===1 && testPosts[0].body.command==='select_teleop','switch submitted motion/enable');
 if (typeof teleopStatusMessage === 'function') {
   Object.assign(testModel.quest,{fresh:true,rate_hz:38,sticks:[0,0,0,0],grips:[0,0],left_tracking:0,right_tracking:0});
   render(testModel); await flush();
   const start=document.querySelector('[data-kind="control"][data-command="start"]');
   assert(!start.disabled,'valid 38 Hz input blocked by obsolete 50 Hz UI gate');
   assert($('control-source-status').textContent.includes('长按'),'missing explicit-start instructions');
   testModel.quest.left_tracking=2;render(testModel);
   assert(start.disabled && $('control-source-status').textContent.includes('左手柄追踪无效'),'invalid tracking not explained or blocked');
   testModel.quest.left_tracking=0;testModel.control.vr_fresh=false;render(testModel);
   assert(start.disabled && $('control-source-status').textContent.includes('数据中断'),'stale input not explained or blocked');
   testModel.control.vr_fresh=true;render(testModel);
   assert(!start.disabled,'fresh tracking did not restore explicit-start availability');
   assert(testPosts.length===1,'readiness recovery automatically started motion');
 }
 consoleButton.click(); await flush();
 assert(consoleButton.getAttribute('aria-pressed')==='true','console selection missing');
 assert(!$('joint-execute').disabled && !$('keyboard-start').disabled && !$('lift-position-enable').disabled,'console did not restore controls');
 assert(testPosts.length===2 && testPosts[1].body.command==='select_console','switch issued extra command');
 assert(testFailures.length===0,'browser errors: '+testFailures);
 document.body.dataset.testStatus='passed';
 document.body.dataset.testResults='explicit source switch; connected Quest does not occupy console; no automatic movement';
})().catch(error=>{document.body.dataset.testStatus='failed';document.body.dataset.testResults=String(error.stack||error);});
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assets", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    html = re.sub(
        r"<script\b[^>]*src=[^>]*></script>", "", (args.assets / "index.html").read_text()
    )
    scripts = [
        fixture.BOOT + MODEL,
        (args.assets / "app.js").read_text(),
        (args.assets / "joint-control.js").read_text(),
        CHECKS,
    ]
    html = html.replace(
        "</body>", "".join("<script>" + script + "</script>" for script in scripts) + "</body>"
    )
    with tempfile.TemporaryDirectory(prefix="authority-browser-") as directory:
        page = Path(directory) / "test.html"
        page.write_text(html)
        result = subprocess.run(
            [
                "/opt/google/chrome/chrome",
                "--headless=new",
                "--no-sandbox",
                "--disable-gpu",
                "--no-proxy-server",
                "--virtual-time-budget=5000",
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
        detail = re.search(r'data-test-results="([^"]*)"', result.stdout)
        raise SystemExit(detail.group(1) if detail else result.stderr[-2000:])
    print("PASS: explicit authority selection, all console actuators and no automatic motion")


if __name__ == "__main__":
    main()
