"""Bounded asynchronous execution evidence; recording failure stops the producer.

The control thread only snapshots and enqueues. A separate writer serializes,
flushes and archives decoded multipart RGB-D bodies without changing timestamps.
"""

import copy
import hashlib
import json
import os
import queue
import threading
import time
import uuid
from pathlib import Path


class ExecutionTrace:
    def __init__(self, directory, *, capacity=2048, max_pending_bytes=64 * 1024 * 1024):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / "frames").mkdir()
        (self.directory / "grasp_points").mkdir()
        self.run_id = uuid.uuid4().hex
        self.queue = queue.Queue(maxsize=capacity)
        self.max_pending_bytes = max_pending_bytes
        self.pending_bytes = 0
        self.lock = threading.Lock()
        self.sequence = 0
        self.error = None
        self.closed = False
        self.worker = threading.Thread(target=self._write, daemon=True, name="execution-log")
        self.worker.start()
        self.record("run_started", image_max_age_s=2.0)

    def check(self):
        if self.error is not None:
            raise RuntimeError("Execution logging failed: " + str(self.error)) from self.error
        if self.closed:
            raise RuntimeError("Execution logging already closed")

    def record(self, kind, *, raw_body=None, **detail):
        with self.lock:
            self.check()
            size = len(raw_body) if raw_body is not None else 0
            if self.pending_bytes + size > self.max_pending_bytes:
                self.error = RuntimeError("Execution image log capacity exceeded")
                self.check()
            self.sequence += 1
            row = dict(
                run_id=self.run_id,
                event_sequence=self.sequence,
                wall_time_ns=time.time_ns(),
                local_mono_ns=time.monotonic_ns(),
                kind=kind,
                detail=copy.deepcopy(detail),
            )
            try:
                self.queue.put_nowait((row, raw_body))
            except queue.Full as error:
                self.error = RuntimeError("Execution event log queue full; recording stopped")
                raise self.error from error
            self.pending_bytes += size
            return self.sequence

    def grasp_points(self, rgb, intrinsics, origin_camera, target, metadata, context):
        """Snapshot the exact source frame; rendering and disk IO stay off control threads."""
        return self.record(
            "grasp_points_visualization",
            raw_body=rgb.tobytes(),
            image_shape=list(rgb.shape),
            intrinsics=intrinsics,
            origin_camera=origin_camera,
            target=target,
            source_metadata=metadata,
            paired_context=context,
            generated_wall_time_ns=time.time_ns(),
            timestamp_note="Source/device/robot monotonic clocks are preserved, not host UTC.",
        )

    def rgbd_response(self, headers, body):
        self.record(
            "rgbd_response",
            raw_body=body,
            headers=dict(headers),
            stored_body_encoding="decoded_multipart",
        )

    @staticmethod
    def _json_value(value):
        if hasattr(value, "tolist"):
            return value.tolist()
        raise TypeError("Unsupported execution trace value: " + type(value).__name__)

    def _write(self):
        last_sync = time.monotonic()
        try:
            with (self.directory / "events.jsonl").open("x") as stream:
                while True:
                    item = self.queue.get()
                    if item is None:
                        stream.flush()
                        os.fsync(stream.fileno())
                        return
                    row, body = item
                    original_body_size = len(body) if body is not None else 0
                    if row["kind"] == "grasp_points_visualization":
                        from wheel_execution.grasp_visualization import render_grasp_svg

                        body = render_grasp_svg(body, row["detail"])
                        stem = f"grasp_points/{row['event_sequence']:09d}"
                        (self.directory / (stem + ".svg")).write_bytes(body)
                        (self.directory / (stem + ".json")).write_text(
                            json.dumps(row, allow_nan=False, default=self._json_value, indent=2)
                        )
                        row["artifact"] = dict(
                            file=stem + ".svg",
                            metadata=stem + ".json",
                            sha256=hashlib.sha256(body).hexdigest(),
                        )
                        body = None
                    if body is not None:
                        name = f"frames/{row['event_sequence']:09d}.multipart"
                        with (self.directory / name).open("xb") as frame:
                            frame.write(body)
                        row["artifact"] = dict(
                            file=name, bytes=len(body), sha256=hashlib.sha256(body).hexdigest()
                        )
                    stream.write(json.dumps(row, allow_nan=False, default=self._json_value) + "\n")
                    stream.flush()
                    if time.monotonic() - last_sync >= 1:
                        os.fsync(stream.fileno())
                        last_sync = time.monotonic()
                    if original_body_size:
                        with self.lock:
                            self.pending_bytes -= original_body_size
        except BaseException as error:
            self.error = error

    def close(self):
        if self.closed:
            if self.error is not None:
                raise RuntimeError("Execution logging failed: " + str(self.error)) from self.error
            return
        # Producers must be stopped by the caller before draining the writer.
        with self.lock:
            self.closed = True
        if self.worker.is_alive():
            try:
                self.queue.put(None, timeout=2)
            except queue.Full as error:
                self.error = error
            self.worker.join(timeout=5)
            if self.worker.is_alive():
                self.error = RuntimeError("Execution log writer did not drain")
        if self.error is not None:
            raise RuntimeError("Execution logging failed: " + str(self.error)) from self.error
