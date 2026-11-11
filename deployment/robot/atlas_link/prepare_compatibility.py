#!/usr/bin/env python3
"""Prepare scoped Atlas keyboard support from the actual deployed carrier sources."""

import argparse
import hashlib
import json
from pathlib import Path


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError(f"Expected one source anchor: {before[:100]!r}")
    return text.replace(before, after, 1)


def patch_header(text):
    return replace_once(
        text,
        "  bool navigation_{false};",
        """  bool navigation_{false};
  bool scoped_keyboard_{false};
  double manual_linear_limit_{.50};
  std::string sdk_owner_;
  unsigned long long sdk_sequence_{0};""",
    )


def patch_carrier(text):
    text = replace_once(text, "#include <sstream>", "#include <sstream>\n#include <random>")
    text = replace_once(
        text,
        "  connected_=true;ready_=false;",
        """  connected_=true;ready_=false;
  const bool shared=flag(doc["base_status"],"shared_enable");
  scoped_keyboard_=shared && flag(doc,"scoped_web_keyboard");
  manual_linear_limit_=.50;
  if(scoped_keyboard_) {
    const auto limit=doc["base_status"]["web_keyboard_linear_limit"];
    if(!limit || !std::isfinite(limit.as<double>()) || limit.as<double>()<=0 || limit.as<double>()>.50)
      throw std::runtime_error("SDK 网页速度上限无效");
    manual_linear_limit_=limit.as<double>();
  }""",
    )
    text = replace_once(
        text,
        '  else if(flag(bs,"shared_enable")||flag(bs,"external_control")) reason_="底盘由 Quest/外部控制器管理，请先退出该模式";',
        """  else if(flag(bs,"external_control")) reason_="底盘由外部控制器管理，请先退出该模式";
  else if(shared && (!scoped_keyboard_ || !flag(bs,"web_keyboard_control"))) reason_="机器人中继需更新以支持当前 SDK 网页控制协议";
  else if(shared && (!bs["control_source"] || bs["control_source"].as<std::string>()!="console")) reason_="请先在机器人控制台选择控制台权限";
  else if(shared && !armed_ && !flag(bs,"keyboard_takeover_available")) reason_="底盘协调器未允许网页接管，请先停止现有控制";""",
    )
    text = replace_once(
        text,
        'request("/api/keyboard","{\\"command\\":\\"stop\\"}");}catch(...) {}}',
        'request("/api/keyboard","{\\"command\\":\\"stop\\",\\"detail\\":"+quoted(sdk_owner_)+"}");}catch(...) {}}\n  sdk_owner_.clear();',
    )
    text = replace_once(
        text,
        '<<(lift?.50:.25)<<",\\"angular_limit\\":"',
        '<<(lift?manual_linear_limit_:.25)<<",\\"angular_limit\\":"',
    )
    text = replace_once(
        text,
        "        // Never retry an uncertain ARM: revoke it and require an explicit new acquire.",
        """        sdk_owner_.clear();sdk_sequence_=0;
        if(scoped_keyboard_ && !navigation) {
          std::random_device random;
          sdk_owner_="atlas:"+std::to_string(random())+":"+std::to_string(random());
        }
        // Never retry an uncertain ARM: revoke it and require an explicit new acquire.""",
    )
    text = replace_once(
        text,
        '            "{\\"command\\":\\"navigation_start\\"}":"{\\"command\\":\\"keyboard_start\\"}"));',
        '            "{\\"command\\":\\"navigation_start\\"}":"{\\"command\\":\\"keyboard_start\\",\\"detail\\":"+quoted(sdk_owner_)+"}"));',
    )
    text = replace_once(
        text,
        '          try {request("/api/keyboard","{\\"command\\":\\"stop\\"}");}catch(...) {}',
        '          try {request("/api/keyboard","{\\"command\\":\\"stop\\",\\"detail\\":"+quoted(sdk_owner_)+"}");}catch(...) {}\n          sdk_owner_.clear();',
    )
    text = replace_once(
        text,
        "    const double linear_limit=navigation_?kNavigationLinearLimit:.50;",
        "    const double linear_limit=navigation_?kNavigationLinearLimit:manual_linear_limit_;",
    )
    text = replace_once(
        text,
        '            else body<<"{\\"command\\":\\"keyboard\\",\\"detail\\":\\"["<<x_/.50<<","<<y_/.50<<","<<yaw_/.30<<"]\\"}";',
        """            else if(scoped_keyboard_) {
              std::ostringstream detail;
              detail<<"{\\"owner\\":"<<quoted(sdk_owner_)<<",\\"sequence\\":"<<++sdk_sequence_
                    <<",\\"velocity\\":["<<x_/manual_linear_limit_<<","<<y_/manual_linear_limit_<<","<<yaw_/.30<<"]}";
              body<<"{\\"command\\":\\"keyboard\\",\\"detail\\":"<<quoted(detail.str())<<"}";
            } else body<<"{\\"command\\":\\"keyboard\\",\\"detail\\":\\"["<<x_/.50<<","<<y_/.50<<","<<yaw_/.30<<"]\\"}";""",
    )
    text = replace_once(
        text,
        '          if(!flag(ack,"success")) return error(503,"SDK 未确认键盘使能");',
        '          if(!flag(ack,"success")) throw std::runtime_error("SDK 未确认键盘使能: "+(ack["message"]?ack["message"].as<std::string>():"未提供原因"));',
    )
    return text.replace("quoted(", "json_quote(")


def patch_transport(text):
    return replace_once(
        text,
        '      {"control_transport","local_ros2"},',
        """      {"control_transport","local_ros2"},
      {"scoped_web_keyboard",true},""",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("work", type=Path)
    args = parser.parse_args()
    patches = {
        "include/luxi_lift_robot/carrier.hpp": patch_header,
        "src/carrier.cpp": patch_carrier,
        "src/sdk_ros.cpp": patch_transport,
    }
    receipt = {}
    for relative, patch in patches.items():
        source = args.source / relative
        before = source.read_text()
        after = patch(before)
        for directory, content in [("original", before), ("candidate", after)]:
            path = args.work / directory / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        receipt[relative] = {
            "original_sha256": hashlib.sha256(before.encode()).hexdigest(),
            "candidate_sha256": hashlib.sha256(after.encode()).hexdigest(),
        }
    (args.work / "manifest.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(args.work)


if __name__ == "__main__":
    main()
