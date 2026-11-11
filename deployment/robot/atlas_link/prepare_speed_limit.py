#!/usr/bin/env python3
"""Prepare a configurable web speed cap using live SDK sources and service settings."""

import argparse
import hashlib
import json
from pathlib import Path

from prepare_compatibility import replace_once


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("work", type=Path)
    args = parser.parse_args()
    replacements = {}
    relative = "Project/device_adapters/src/r680_adapter.cpp"
    source = (args.root / relative).read_bytes().decode()
    candidate = replace_once(
        source,
        "  static constexpr double web_keyboard_linear_limit_ = .05;",
        "  double web_keyboard_linear_limit_ = .05;",
    )
    candidate = replace_once(
        candidate,
        '    gate_.mapping.linear = declare_parameter("max_linear", .2);',
        """    gate_.mapping.linear = declare_parameter("max_linear", .2);
    web_keyboard_linear_limit_ = declare_parameter("web_keyboard_linear_limit", .05);
    if (!std::isfinite(web_keyboard_linear_limit_) || web_keyboard_linear_limit_ <= 0 ||
        web_keyboard_linear_limit_ > .50)
      throw std::invalid_argument("web_keyboard_linear_limit must be in (0, 0.50] m/s");""",
    )
    replacements[relative] = (source, candidate)
    relative = "Project/luxi_web_ctrl/web/app.js"
    source = (args.root / relative).read_bytes().decode()
    candidate = replace_once(
        source,
        "  const s = state, bs = s?.base_status;",
        """  const s = state, bs = s?.base_status;
  if ($('web-keyboard-speed-limit')) text('web-keyboard-speed-limit', Number(bs?.web_keyboard_linear_limit ?? .05).toFixed(2));""",
    )
    candidate = replace_once(
        candidate,
        "log('WASD / Q/E 已启用，限速 0.05 m/s');",
        "log('WASD / Q/E 已启用，限速 ' + Number(state?.base_status?.web_keyboard_linear_limit ?? .05).toFixed(2) + ' m/s');",
    )
    if "\r\n" in source:
        candidate = candidate.replace("\r\n", "\n").replace("\n", "\r\n")
    replacements[relative] = (source, candidate)
    relative = "Project/luxi_web_ctrl/web/index.html"
    source = (args.root / relative).read_bytes().decode()
    replacements[relative] = (
        source,
        replace_once(
            source,
            "平移速度上限 0.05 m/s；",
            '平移速度上限 <span id="web-keyboard-speed-limit">—</span> m/s；',
        ),
    )
    service = Path("/etc/systemd/system/luxi-base.service.d/quest-teleop.conf")
    service_source = service.read_text()
    service_candidate = replace_once(
        service_source,
        '-p max_linear:=0.5"',
        '-p max_linear:=0.5 -p web_keyboard_linear_limit:=0.5"',
    )
    args.work.mkdir(parents=True, exist_ok=True)
    manifest = {}
    for relative, (source, candidate) in replacements.items():
        for directory, content in [("original", source), ("candidate", candidate)]:
            destination = args.work / directory / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(content.encode())
        manifest[relative] = {
            "before_sha256": hashlib.sha256(source.encode()).hexdigest(),
            "candidate_sha256": hashlib.sha256(candidate.encode()).hexdigest(),
        }
    (args.work / "service.before").write_text(service_source)
    (args.work / "service.candidate").write_text(service_candidate)
    (args.work / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(args.work)


if __name__ == "__main__":
    main()
