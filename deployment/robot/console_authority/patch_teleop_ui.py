#!/usr/bin/env python3
"""Align UI readiness with existing SDK checks; never restart device services."""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

from prepare_sdk import replace

STATUS = """function teleopStatusMessage() {
  const c = state?.control, q = state?.quest;
  if (!q?.fresh || !c?.vr_fresh) return '遥操权限已选中，但手柄数据中断。请恢复 Quest 应用的数据发送，再启用遥操。';
  const invalid = [q.left_tracking >= 2 ? '左手柄' : '', q.right_tracking >= 2 ? '右手柄' : ''].filter(Boolean);
  if (invalid.length) return invalid.join('、') + '追踪无效。请恢复手柄追踪，再松开 X 后长按约 1.5 秒启用遥操。';
  if (state?.recorder?.failure_menu) return '录制失败菜单正在阻止启用，请先关闭该菜单。';
  if (c.system_phase === 'active') return '遥操作已启用。控制台停止按钮始终可用。';
  if (!['disabled','stopped'].includes(c.system_phase)) return '遥操正在启动：' + (systemPhases[c.system_phase] || c.system_phase);
  return '遥操权限已选中，尚未启用。手柄回中立，松开 X 后重新长按约 1.5 秒，或点击“启用全部遥操作”。';
}
"""


def patch(text):
    text = replace(
        text,
        "s.quest?.fresh && s.quest.rate_hz >= 50 && s.control?.fresh;",
        "s.quest?.fresh && s.quest.left_tracking < 2 && s.quest.right_tracking < 2 && s.control?.vr_fresh && s.control.quest_window_samples >= 3 && s.control?.fresh;",
    )
    text = replace(text, "function controlSourceUI() {", STATUS + "\nfunction controlSourceUI() {")
    return replace(
        text,
        "    '遥操持有权限：仍需显式启用遥操作才会运动。控制台停止按钮始终可用。'));",
        "    teleopStatusMessage()));",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("work", type=Path)
    parser.add_argument("--install", action="store_true")
    args = parser.parse_args()
    source = args.root / "Project/luxi_web_ctrl/web/app.js"
    installed = args.root / "build/install/luxi_web_ctrl/share/luxi_web_ctrl/web/app.js"
    if args.install:
        manifest = json.loads((args.work / "manifest.json").read_text())
        candidate = (args.work / "app.js").read_bytes()
        tested = json.loads((args.work / "browser-tests-passed.json").read_text())
        if hashlib.sha256(candidate).hexdigest() != tested["candidate_sha256"]:
            raise RuntimeError("Candidate changed after browser verification")
        for target in [source, installed]:
            if hashlib.sha256(target.read_bytes()).hexdigest() != manifest["before_sha256"]:
                raise RuntimeError("Live webpage changed; preserve and rebase first")
        for target in [source, installed]:
            target = target.resolve()
            temporary = target.with_name(target.name + ".teleop-readiness-new")
            shutil.copy2(args.work / "app.js", temporary)
            temporary.chmod(target.stat().st_mode)
            temporary.replace(target)
        (args.work / "install-receipt.json").write_text(
            json.dumps(
                {
                    "candidate_sha256": tested["candidate_sha256"],
                    "changed": [str(source), str(installed)],
                    "services_restarted": [],
                    "motion_commands_sent": False,
                },
                indent=2,
            )
            + "\n"
        )
        print("Updated webpage only; no service restart or motion command")
        return
    before = source.read_bytes()
    if installed.read_bytes() != before:
        raise RuntimeError("Installed webpage has separate changes")
    updated = patch(before.decode().replace("\r\n", "\n"))
    candidate = updated.replace("\n", "\r\n").encode() if b"\r\n" in before else updated.encode()
    args.work.mkdir(parents=True, exist_ok=False)
    (args.work / "app.js.before").write_bytes(before)
    (args.work / "app.js").write_bytes(candidate)
    (args.work / "manifest.json").write_text(
        json.dumps(
            {
                "before_sha256": hashlib.sha256(before).hexdigest(),
                "candidate_sha256": hashlib.sha256(candidate).hexdigest(),
            },
            indent=2,
        )
        + "\n"
    )
    print(args.work)


if __name__ == "__main__":
    main()
