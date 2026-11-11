"""Bounded raw feedback diagnostics, including rejected responses and HTTP timing."""

import json
import logging
import threading
from logging.handlers import RotatingFileHandler
from pathlib import Path

_LOCK = threading.Lock()
_HANDLERS = {}


def append_diagnostic(path, record):
    path = Path(path).resolve()
    with _LOCK:
        if path not in _HANDLERS:
            path.parent.mkdir(parents=True, exist_ok=True)
            handler = RotatingFileHandler(
                path, maxBytes=8 * 1024 * 1024, backupCount=3, encoding="utf-8"
            )
            handler.setFormatter(logging.Formatter("%(message)s"))
            _HANDLERS[path] = handler
        message = json.dumps(record, ensure_ascii=False, allow_nan=False)
        _HANDLERS[path].handle(
            logging.LogRecord("joint_feedback", logging.INFO, "", 0, message, (), None)
        )


def timing(state, elapsed):
    result = dict(request_ms=elapsed * 1000)
    try:
        now = int(state["server_mono_ns"])
        for name in ("arms", "arm_startup"):
            age = (now - int(state[name]["receive_mono_ns"])) / 1e6
            result[name + "_cache_ms"] = age
            result[name + "_effective_ms"] = age + elapsed * 1000
        result["max_joint_effective_ms"] = (
            max(map(float, state["arms"]["age_ns"])) / 1e6 + result["arms_effective_ms"]
        )
        result["pair_skew_ms"] = (
            abs(
                int(state["arms"]["receive_mono_ns"]) - int(state["arm_startup"]["receive_mono_ns"])
            )
            / 1e6
        )
    except (KeyError, TypeError, ValueError):
        pass
    return result
