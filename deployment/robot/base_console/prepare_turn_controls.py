#!/usr/bin/env python3
"""Add Q/E and hold buttons to the existing web base controls."""

import argparse
import hashlib
import json
from pathlib import Path

from prepare_sdk import replace_once


def patch_web(text: str) -> str:
    text = replace_once(text, "keyboardPending = false,", "keyboardPending = 0,")
    text = replace_once(
        text,
        "for (const key of ['w','a','s','d']) $('key-'+key).classList.toggle('pressed', heldKeys.has('Key'+key.toUpperCase()));",
        """for (const key of ['w','a','s','d','q','e']) {
    const button = $('key-'+key);
    if (!button) continue;
    const pressed = heldKeys.has('Key'+key.toUpperCase());
    button.classList.toggle('pressed', pressed);
    button.setAttribute('aria-pressed', String(pressed));
    button.disabled = !keyboardActive;
  }""",
    )
    text = replace_once(
        text,
        "async function keyboardTick() {\n  if (!keyboardActive || keyboardPending) return;\n  keyboardPending = true;",
        """async function keyboardTick(priority = false) {
  if (!keyboardActive || (keyboardPending && !priority)) return;
  keyboardPending += 1;""",
    )
    text = replace_once(
        text,
        "Number(heldKeys.has('KeyD')), 0]}));",
        "Number(heldKeys.has('KeyD')), Number(heldKeys.has('KeyQ'))-Number(heldKeys.has('KeyE'))]}));",
    )
    text = replace_once(
        text,
        "finally { keyboardPending = false; }",
        "finally { keyboardPending -= 1; }",
    )
    text = replace_once(
        text,
        "if (['KeyW','KeyA','KeyS','KeyD'].includes(e.code))",
        "if (['KeyW','KeyA','KeyS','KeyD','KeyQ','KeyE'].includes(e.code))",
    )
    text = replace_once(
        text,
        "window.addEventListener('keyup', e => { if (heldKeys.delete(e.code)) { e.preventDefault(); keyboardUI(); keyboardTick(); } });",
        """window.addEventListener('keyup', e => { if (heldKeys.delete(e.code)) { e.preventDefault(); keyboardUI(); keyboardTick(true); } });
['w','a','s','d','q','e'].forEach(name => {
  const button = $('key-'+name), key = 'Key'+name.toUpperCase();
  if (!button) return;
  button.addEventListener('pointerdown', e => {
    if (!keyboardActive || (e.button !== undefined && e.button !== 0)) return;
    e.preventDefault();
    heldKeys.add(key); keyboardUI(); keyboardTick();
    try { button.setPointerCapture(e.pointerId); } catch {}
  });
  const release = e => {
    if (!heldKeys.delete(key)) return;
    e.preventDefault(); keyboardUI(); keyboardTick(true);
  };
  for (const event of ['pointerup','pointercancel','lostpointercapture'])
    button.addEventListener(event, release);
});""",
    )
    return text.replace("WASD", "WASD / Q/E")


def patch_html(text: str) -> str:
    before = '<div class="keyboard-keys" aria-label="键盘方向"><kbd id="key-w">W ↑ 前进</kbd><kbd id="key-a">A ← 左移</kbd><kbd id="key-s">S ↓ 后退</kbd><kbd id="key-d">D → 右移</kbd></div>'
    buttons = []
    for key, label in [
        ("w", "W ↑ 前进"),
        ("a", "A ← 左移"),
        ("s", "S ↓ 后退"),
        ("d", "D → 右移"),
        ("q", "Q ↶ 原地左转"),
        ("e", "E ↷ 原地右转"),
    ]:
        buttons.append(
            f'<button type="button" id="key-{key}" class="direction-key" '
            f'data-direction-key="Key{key.upper()}" aria-pressed="false" disabled>{label}</button>'
        )
    text = replace_once(
        text,
        before,
        '<div class="keyboard-keys" aria-label="按住键盘或页面按钮控制底盘">'
        + "".join(buttons)
        + "</div>",
    )
    text = replace_once(
        text,
        "测试速度上限 0.05 m/s。",
        "平移速度上限 0.05 m/s；原地转向上限 0.30 rad/s（约 17°/s）。按住 Q/E 或页面左转、右转按钮即可转向，松开停车。",
    )
    text = replace_once(text, "WASD / CH3", "WASD + Q/E / CH3")
    return (
        text.replace("打开开关即可使用 WASD", "打开开关即可使用 WASD 和 Q/E")
        .replace("按住 WASD 行驶", "按住 WASD 平移、Q/E 转向")
        .replace("松开 WASD 后再按", "松开方向键后再按")
    )


def patch_css(text: str) -> str:
    text = replace_once(text, ".keyboard-keys kbd {", ".keyboard-keys .direction-key {")
    text = replace_once(
        text,
        ".keyboard-keys kbd.pressed {",
        ".keyboard-keys .direction-key.pressed {",
    )
    return text + "\n.keyboard-keys .direction-key { touch-action:none; user-select:none; }\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    patches = {
        "Project/luxi_web_ctrl/web/app.js": patch_web,
        "Project/luxi_web_ctrl/web/index.html": patch_html,
        "Project/luxi_web_ctrl/web/style.css": patch_css,
    }
    prepared = []
    for name, patch in patches.items():
        raw = (args.root / name).read_bytes()
        updated = patch(raw.decode().replace("\r\n", "\n"))
        if b"\r\n" in raw:
            updated = updated.replace("\n", "\r\n")
        prepared.append((name, raw, updated.encode()))
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {}
    for name, before, after in prepared:
        for directory, raw in [("before", before), ("candidate", after)]:
            target = args.output / directory / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(raw)
        manifest[name] = {
            "before_sha256": hashlib.sha256(before).hexdigest(),
            "candidate_sha256": hashlib.sha256(after).hexdigest(),
        }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(args.output.resolve())


if __name__ == "__main__":
    main()
