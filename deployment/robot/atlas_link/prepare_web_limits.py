#!/usr/bin/env python3
"""Prepare Atlas UI limits using the actual carrier's reported speed capability."""

import argparse
import shutil
from pathlib import Path

from prepare_compatibility import replace_once


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("work", type=Path)
    parser.add_argument("--node-modules", type=Path)
    args = parser.parse_args()
    candidate = args.work / "web-candidate"
    candidate.mkdir(exist_ok=True)
    for path in args.source.iterdir():
        if path.name in {"node_modules", "dist"}:
            continue
        target = candidate / path.name
        if path.is_dir():
            shutil.copytree(path, target, dirs_exist_ok=True)
        else:
            shutil.copy2(path, target)
    dependencies = args.node_modules or args.source / "node_modules"
    if not dependencies.is_dir():
        raise FileNotFoundError(f"Frontend dependencies unavailable: {dependencies}")
    dependency_link = candidate / "node_modules"
    if dependency_link.is_symlink():
        dependency_link.unlink()
    dependency_link.symlink_to(dependencies, target_is_directory=True)
    store_path = candidate / "src/stores/manualControl.ts"
    store = store_path.read_text()
    store = replace_once(
        store,
        "  const linearLimit = computed(() => isLift.value ? manualSpeed.value : MAX_LINEAR_SPEED);",
        """  const sdkLinearLimit = computed(() => robot.status?.linear_limit ?? 0.05);
  const linearLimit = computed(() => isLift.value ? Math.min(manualSpeed.value, sdkLinearLimit.value) : MAX_LINEAR_SPEED);
  watch(sdkLinearLimit, (limit) => {
    if (manualSpeed.value > limit) manualSpeed.value = limit;
    stopManual();
  }, { immediate: true, flush: "sync" });""",
    )
    # The immediate watcher runs after manual state has been initialized.
    watcher_start = store.index("  watch(sdkLinearLimit,")
    watcher_end = store.index('  }, { immediate: true, flush: "sync" });', watcher_start)
    watcher_end += len('  }, { immediate: true, flush: "sync" });\n')
    watcher = store[watcher_start:watcher_end]
    store = store[:watcher_start] + store[watcher_end:]
    store = replace_once(store, "  watch(manualSpeed,", watcher + "\n  watch(manualSpeed,")
    store_path.write_text(store)
    view_path = candidate / "src/views/DeviceControlView.vue"
    view = replace_once(
        view_path.read_text(),
        'min="0.05" max="0.50" step="0.01"',
        ':min="Math.min(0.05, robot.status?.linear_limit ?? 0.05)" :max="robot.status?.linear_limit ?? 0.05" step="0.01"',
    )
    view = replace_once(
        view,
        "<span>0.50 m/s</span>",
        "<span>{{ (robot.status?.linear_limit ?? 0.05).toFixed(2) }} m/s</span>",
    )
    view_path.write_text(view)
    test_path = candidate / "tests/manual_control.test.mjs"
    tests = replace_once(
        test_path.read_text(),
        'status: {robot_type: lift ? "lift_robot" : "go2"}',
        'status: {robot_type: lift ? "lift_robot" : "go2", linear_limit: .50}',
    )
    tests += """
test("shared SDK capability caps actual requested velocity", () => {
  const f=fixture(true);
  f.robot.status.linear_limit=.05;
  f.control.hasControl=true;
  f.store.manualSpeed=.50;
  f.store.pressDirection('forward');
  assert.equal(f.moves.at(-1).linearX,.05);
  f.store.pressDirection('left');
  const velocity=f.moves.at(-1);
  assert.ok(Math.abs(Math.hypot(velocity.linearX,velocity.linearY)-.05)<1e-8);
});

test("capability reduction stops active input and requires a new press", () => {
  const f=fixture(true);
  f.control.hasControl=true;
  f.store.pressDirection('forward');
  f.robot.status.linear_limit=.05;
  assert.equal(f.moves.at(-1).linearX,0);
  assert.equal(f.store.manualSpeed,.05);
  assert.equal(f.store.manualActive,false);
  assert.equal(f.intervals.size,0);
});
"""
    test_path.write_text(tests)
    print(candidate)


if __name__ == "__main__":
    main()
