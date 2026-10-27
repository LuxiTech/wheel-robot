"""Resolve a recipe relative to its robot profile; never mutate input files."""
from pathlib import Path
import copy
import yaml


def merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_profile(path):
    path = Path(path)
    profile = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(profile, dict):
        raise ValueError('Configuration must be a mapping')
    recipe = profile.pop('recipe', None)
    if recipe is None:
        return profile
    if not isinstance(recipe, str) or not recipe.strip():
        raise ValueError('recipe must be a nonempty relative path')
    recipe_path = Path(recipe)
    if recipe_path.is_absolute():
        raise ValueError('recipe must be relative to the robot profile')
    defaults = yaml.safe_load((path.parent / recipe_path).read_text(encoding='utf-8'))
    if not isinstance(defaults, dict) or 'recipe' in defaults:
        raise ValueError('Recipe must be a mapping without nested recipe includes')
    return merge(defaults, profile)
