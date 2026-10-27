"""Dependency direction, configuration semantics and maintained documentation."""
import ast
from pathlib import Path
import pytest

from wheel_task_core.configuration import load_profile

ROOT = Path(__file__).resolve().parents[1]


def test_core_libraries_do_not_import_runtime_or_each_other():
    common_forbidden = ('rclpy', 'fastapi', 'wheel_sdk_bridge', 'wheel_runtime',
                        'wheel_execution', 'wheel_navigation', 'wheel_gateway',
                        'wheel_perception', 'wheel_planning', 'management_api')
    for package in ('wheel_task_core', 'wheel_perception_core', 'wheel_motion_core'):
        for file in (ROOT / 'packages' / package / package).glob('*.py'):
            imports = []
            for node in ast.walk(ast.parse(file.read_text(encoding='utf-8'))):
                if isinstance(node, ast.Import):
                    imports.extend(alias.name.split('.')[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    imports.append(node.module.split('.')[0])
            forbidden = set(common_forbidden)
            forbidden |= {'wheel_perception_core', 'wheel_motion_core'} - {package}
            assert not forbidden.intersection(imports), str(file)


def test_every_maintained_directory_has_readme():
    missing = []
    for name in ('packages', 'ros2_ws/src', 'apps', 'config', 'deployment', 'tools', 'tests', 'docs'):
        parent = ROOT / name
        folders = [parent] + [p for p in parent.rglob('*') if p.is_dir()]
        for folder in folders:
            if any(part in ('__pycache__', '.pytest_cache', 'node_modules', 'dist', 'build')
                   or part.endswith('.egg-info') for part in folder.relative_to(ROOT).parts):
                continue
            if not (folder / 'README.md').is_file():
                missing.append(str(folder.relative_to(ROOT)))
    assert not missing, missing


def test_robot_profiles_resolve_shared_recipe_without_validating_hardware():
    for name in ('runtime.yaml', 'runtime.live.yaml', 'runtime.container.yaml'):
        cfg = load_profile(ROOT / 'config/robots' / name)
        assert cfg['rules']['width'] == .4
        assert cfg['rules']['height'] == .22
        assert cfg['model']['mesh'] == 'assets/objects/crate/model.obj'
        assert cfg['lift_mapping'] is None
        assert cfg['tcp_hardware_validated'] is False
        assert cfg['object_geometry_validated'] is False


def test_profile_overrides_nested_recipe_and_does_not_modify_it(tmp_path):
    recipe = tmp_path / 'crate.yaml'
    recipe.write_text('rules: {width: 0.4, height: 0.22}\n', encoding='utf-8')
    profile = tmp_path / 'robot.yaml'
    profile.write_text('recipe: crate.yaml\nrules: {width: 0.445}\n', encoding='utf-8')
    assert load_profile(profile)['rules'] == {'width': .445, 'height': .22}
    assert '0.445' not in recipe.read_text(encoding='utf-8')
    recipe.write_text('recipe: another.yaml\n', encoding='utf-8')
    with pytest.raises(ValueError, match='nested'):
        load_profile(profile)
