#!/usr/bin/env python3
"""Offline residual model comparison; fitted joint values never reach hardware."""

import argparse
import hashlib
import json
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.interpolate import RBFInterpolator
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure
from _robot_model import selected_model

configure(ROOT)
from grasp import recalibrate_recorded_head as common
from wheel_motion_core.kinematics import RobotModel
from wheel_motion_core.sdk_kinematics import SDKKinematics


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


@dataclass
class Spec:
    name: str
    camera: str = "none"
    height: int = -1
    joint: str = "none"
    individual: int = -1
    mount: bool = False
    empirical: str = "none"
    pixels: bool = False


def catalog():
    result = [
        Spec("baseline_original"),
        Spec("baseline_URDF_height", camera="URDF_Z"),
        Spec("image_translation_constant", pixels=True),
        Spec("height_constant", height=0),
        Spec("height_linear", height=1),
        Spec("height_quadratic", height=2),
        Spec("global_translation", camera="translation"),
        Spec("global_rigid", camera="rigid"),
        Spec("camera_and_board_mount", camera="rigid", mount=True),
    ]
    result += [Spec(f"left_J{i + 1}_offset", joint="individual", individual=i) for i in range(7)]
    result += [
        Spec("left_joint_offsets", joint="offset"),
        Spec("left_joint_affine", joint="affine"),
        Spec("left_joint_nonlinear_sine", joint="sine"),
        Spec("camera_and_joint_offsets", camera="rigid", joint="offset"),
        Spec("camera_board_and_joint_offsets", camera="rigid", joint="offset", mount=True),
        Spec("pose_residual_linear_ridge", empirical="linear"),
        Spec("pose_residual_quadratic_ridge", empirical="quadratic"),
        Spec("pose_residual_RBF", empirical="rbf"),
        Spec("pose_residual_RBF_smoothed", empirical="rbf_smooth"),
    ]
    return result


class Context:
    def __init__(self, calibration, sdk):
        self.calibration = calibration
        self.camera = np.array(calibration["T_origin_camera"])
        self.mount = np.array(calibration["T_ee_board"])
        self.reference = calibration["reference_height_um"] * 1e-6
        self.chain = sdk.chains["left"]
        self.joint_names = [f"openarm_left_joint{i + 1}" for i in range(7)]
        resources = selected_model(ROOT)
        geometry = RobotModel(resources["urdf"])
        base = geometry.transform("openarm_left_base_link", {"lift_joint": self.reference + 0.2})
        registration = base @ np.linalg.inv(sdk.transform("openarm_left_base_link"))
        camera_in_origin = np.linalg.inv(registration) @ geometry.transform(
            "head_d455_color_optical_frame", {"lift_joint": self.reference + 0.2}
        )
        self.URDF_Z_delta = float(camera_in_origin[2, 3] - self.camera[2, 3])

    def fk(self, q):
        count = len(q)
        rotation = np.broadcast_to(np.eye(3), (count, 3, 3)).copy()
        translation = np.zeros((count, 3))
        for node in self.chain:
            translation += np.einsum("nij,j->ni", rotation, node["translation_m"])
            if "joint" in node:
                joint = node["joint"]
                index = self.joint_names.index(joint["name"])
                motion = Rotation.from_rotvec(
                    q[:, index, None] * np.array(joint["axis"])
                ).as_matrix()
                rotation = rotation @ motion
            if node["name"] == "left_ee_control_point":
                break
        return rotation, translation

    def data(self, pairs):
        return dict(
            pairs=pairs,
            q=np.array([p["q"][8:15] for p in pairs]),
            h=np.array([p["height_m"] for p in pairs]),
            rays=np.array([p["rays"] for p in pairs]),
            focal=np.array([[p["intrinsics"]["K"][0], p["intrinsics"]["K"][4]] for p in pairs]),
        )

    def features(self, data):
        return np.column_stack([data["q"], (data["h"] - self.reference) / 0.1])


class Model:
    def __init__(self, context, spec):
        self.ctx, self.spec = context, spec
        self.slices = {}
        self.names = []
        self.lower, self.upper = [], []
        self.value = None
        self.empirical_state = None
        if spec.camera == "rigid":
            self.add("camera_rotation", 3, np.deg2rad(5), "rad")
        if spec.camera in ("rigid", "translation"):
            self.add("camera_translation", 3, 0.06, "m")
        if spec.height >= 0:
            self.add(
                "height_coefficients", spec.height + 1, 0.10, "m; polynomial argument=(h-h0)/0.1m"
            )
        if spec.joint == "individual":
            self.add("joint_individual", 1, np.deg2rad(10), "rad")
        elif spec.joint != "none":
            self.add("joint_offsets", 7, np.deg2rad(10), "rad")
            if spec.joint in ("affine", "sine"):
                self.add("joint_gains", 7, 0.05, "dimensionless")
            if spec.joint == "sine":
                self.add("joint_sine_amplitudes", 7, np.deg2rad(5), "rad")
        if spec.mount:
            self.add("board_rotation", 3, np.deg2rad(8), "rad")
            self.add("board_translation", 3, 0.06, "m")
        if spec.pixels:
            self.add("image_shift", 2, 50.0, "px")

    def add(self, name, size, bound, unit):
        start = len(self.lower)
        self.slices[name] = slice(start, start + size)
        self.names += [f"{name}[{i}] ({unit})" for i in range(size)]
        self.lower += [-bound] * size
        self.upper += [bound] * size

    def at(self, parameters, name):
        return parameters[self.slices[name]]

    def polynomial(self, x, quadratic=False):
        basis = [np.ones(len(x)), *x.T]
        if quadratic:
            basis += [x[:, i] * x[:, j] for i in range(x.shape[1]) for j in range(i, x.shape[1])]
        return np.column_stack(basis)

    def predict(self, data, parameters=None):
        parameters = self.value if parameters is None else parameters
        q = data["q"].copy()
        camera_r = self.ctx.camera[:3, :3].copy()
        camera_t = np.broadcast_to(self.ctx.camera[:3, 3], (len(q), 3)).copy()
        camera_t[:, 2] -= data["h"] - self.ctx.reference
        mount_r, mount_t = self.ctx.mount[:3, :3].copy(), self.ctx.mount[:3, 3].copy()
        if self.spec.camera == "URDF_Z":
            camera_t[:, 2] += self.ctx.URDF_Z_delta
        if "camera_rotation" in self.slices:
            camera_r = (
                Rotation.from_rotvec(self.at(parameters, "camera_rotation")).as_matrix() @ camera_r
            )
        if "camera_translation" in self.slices:
            camera_t += self.at(parameters, "camera_translation")
        if self.spec.height >= 0:
            x = (data["h"] - self.ctx.reference) / 0.1
            camera_t[:, 2] += np.polynomial.polynomial.polyval(
                x, self.at(parameters, "height_coefficients")
            )
        if self.spec.joint == "individual":
            q[:, self.spec.individual] += self.at(parameters, "joint_individual")[0]
        elif self.spec.joint != "none":
            q += self.at(parameters, "joint_offsets")
            if "joint_gains" in self.slices:
                q += data["q"] * self.at(parameters, "joint_gains")
            if "joint_sine_amplitudes" in self.slices:
                q += np.sin(data["q"]) * self.at(parameters, "joint_sine_amplitudes")
        if self.spec.mount:
            mount_r = (
                mount_r @ Rotation.from_rotvec(self.at(parameters, "board_rotation")).as_matrix()
            )
            mount_t += self.at(parameters, "board_translation")
        arm_r, arm_t = self.ctx.fk(q)
        if self.spec.empirical != "none":
            features = (self.ctx.features(data) - self.feature_mean) / self.feature_scale
            if self.spec.empirical.startswith("rbf"):
                delta = self.rbf(features)
            else:
                delta = (
                    self.polynomial(features, self.spec.empirical == "quadratic")
                    @ self.coefficients
                )
            arm_r = Rotation.from_rotvec(delta[:, :3]).as_matrix() @ arm_r
            arm_t += delta[:, 3:]
        wrist_points = common.OBJECT_POINTS @ mount_r.T + mount_t
        origin_points = np.einsum("nij,kj->nki", arm_r, wrist_points) + arm_t[:, None, :]
        camera_points = (origin_points - camera_t[:, None, :]) @ camera_r
        normalized = camera_points[:, :, :2] / np.maximum(camera_points[:, :, 2:], 0.03)
        residual = (normalized - data["rays"]) * data["focal"][:, None, :]
        if self.spec.pixels:
            residual += self.at(parameters, "image_shift")
        return dict(
            residual=residual,
            camera_points=camera_points,
            arm_R=arm_r,
            arm_t=arm_t,
            camera_R=camera_r,
            camera_t=camera_t,
            mount_R=mount_r,
            mount_t=mount_t,
            virtual_q=q,
        )

    def fit(self, data, max_nfev):
        started = time.monotonic()
        counts = Counter((p["height_group"], p["group"]) for p in data["pairs"])
        weights = np.repeat(
            [1 / counts[(p["height_group"], p["group"])] for p in data["pairs"]], 176
        )
        if self.spec.empirical != "none":
            self.fit_empirical(data, counts)
            return dict(
                success=True,
                elapsed_s=time.monotonic() - started,
                estimated_degrees_of_freedom=self.empirical_state["coefficient_count"],
                data_identifiability="empirical fit; coefficients are not hardware calibration",
            )
        self.value = np.zeros(len(self.names))
        if not len(self.value):
            return dict(
                success=True,
                parameters=[],
                data_rank=0,
                parameter_count=0,
                elapsed_s=time.monotonic() - started,
            )

        def loss(squared):
            root = np.sqrt(1 + squared)
            return np.vstack([2 * (root - 1) * weights, weights / root, -0.5 * weights / root**3])

        result = least_squares(
            lambda parameters: self.predict(data, parameters)["residual"].ravel(),
            self.value,
            bounds=(self.lower, self.upper),
            loss=loss,
            f_scale=1.5,
            x_scale="jac",
            max_nfev=max_nfev,
            ftol=1e-8,
            xtol=1e-8,
            gtol=1e-8,
        )
        self.value = result.x
        singular = np.linalg.svd(result.jac, compute_uv=False)
        rank = (
            int(np.count_nonzero(singular > max(singular[0] * 1e-7, 1e-8))) if len(singular) else 0
        )
        return dict(
            success=bool(result.success),
            message=str(result.message),
            nfev=int(result.nfev),
            elapsed_s=time.monotonic() - started,
            cost=float(result.cost),
            parameters=result.x.tolist(),
            parameter_names=self.names,
            parameter_count=len(result.x),
            data_rank=rank,
            jacobian_singular_values=singular.tolist(),
            rank_deficient=rank < len(result.x),
            at_bounds=[
                self.names[i]
                for i, value in enumerate(result.x)
                if min(value - self.lower[i], self.upper[i] - value) < 1e-5
            ],
        )

    def fit_empirical(self, data, counts):
        features = self.ctx.features(data)
        self.feature_mean = features.mean(axis=0)
        self.feature_scale = np.maximum(features.std(axis=0), 0.05)
        features = (features - self.feature_mean) / self.feature_scale
        arm_r, arm_t = self.ctx.fk(data["q"])
        observed = []
        for pair in data["pairs"]:
            camera = self.ctx.camera.copy()
            camera[2, 3] -= pair["height_m"] - self.ctx.reference
            observed.append(camera @ np.array(pair["B"]) @ np.linalg.inv(self.ctx.mount))
        observed = np.array(observed)
        target = np.column_stack(
            [
                Rotation.from_matrix(observed[:, :3, :3] @ np.swapaxes(arm_r, 1, 2)).as_rotvec(),
                observed[:, :3, 3] - arm_t,
            ]
        )
        if self.spec.empirical.startswith("rbf"):
            smoothing = 0.001 if self.spec.empirical == "rbf" else 1.0
            self.rbf = RBFInterpolator(
                features, target, smoothing=smoothing, kernel="gaussian", epsilon=0.5, degree=0
            )
            state = dict(
                training_features=features.tolist(),
                training_targets=target.tolist(),
                smoothing=smoothing,
                kernel="gaussian",
                epsilon=0.5,
                coefficient_count=len(target) * 6,
            )
        else:
            basis = self.polynomial(features, self.spec.empirical == "quadratic")
            weights = np.array([1 / counts[(p["height_group"], p["group"])] for p in data["pairs"]])
            penalty = np.eye(basis.shape[1]) * 0.01
            penalty[0, 0] = 0
            self.coefficients = np.linalg.solve(
                basis.T @ (weights[:, None] * basis) + penalty,
                basis.T @ (weights[:, None] * target),
            )
            state = dict(
                coefficients=self.coefficients.tolist(),
                ridge=0.01,
                basis="quadratic" if self.spec.empirical == "quadratic" else "linear",
                coefficient_count=int(self.coefficients.size),
            )
        state.update(
            feature_mean=self.feature_mean.tolist(),
            feature_scale=self.feature_scale.tolist(),
            target="origin-frame wrist rotation vector and translation closure residual",
        )
        self.empirical_state = state

    def export(self):
        return dict(
            spec=self.spec.__dict__,
            parameters=None if self.value is None else self.value.tolist(),
            parameter_names=self.names,
            empirical=self.empirical_state,
        )

    @classmethod
    def restore(cls, context, exported):
        model = cls(context, Spec(**exported["spec"]))
        model.value = (
            np.array(exported["parameters"]) if exported["parameters"] is not None else None
        )
        state = exported["empirical"]
        if state is not None:
            model.empirical_state = state
            model.feature_mean = np.array(state["feature_mean"])
            model.feature_scale = np.array(state["feature_scale"])
            if model.spec.empirical.startswith("rbf"):
                model.rbf = RBFInterpolator(
                    np.array(state["training_features"]),
                    np.array(state["training_targets"]),
                    smoothing=state["smoothing"],
                    kernel=state["kernel"],
                    epsilon=state["epsilon"],
                    degree=0,
                )
            else:
                model.coefficients = np.array(state["coefficients"])
        return model


def evaluate(model, data):
    import pyrealsense2 as rs

    prediction = model.predict(data)
    rows = []
    for i, pair in enumerate(data["pairs"]):
        if np.any(prediction["camera_points"][i, :, 2] <= 0.03):
            raise ValueError("Predicted board behind or too close to camera")
        intrinsics = common.real_sense_intrinsics(pair["intrinsics"])
        projected = np.array(
            [
                rs.rs2_project_point_to_pixel(intrinsics, point.tolist())
                for point in prediction["camera_points"][i]
            ]
        )
        if model.spec.pixels:
            projected += model.at(model.value, "image_shift")
        distances = np.linalg.norm(projected - np.array(pair["corners"]), axis=1)
        predicted_r = prediction["arm_R"][i] @ prediction["mount_R"]
        predicted_t = prediction["arm_R"][i] @ prediction["mount_t"] + prediction["arm_t"][i]
        observed = np.eye(4)
        observed[:3, :3] = prediction["camera_R"]
        observed[:3, 3] = prediction["camera_t"][i]
        observed = observed @ np.array(pair["B"])
        rows.append(
            dict(
                frame_index=pair["frame_index"],
                group=pair["group"],
                height_group=pair["height_group"],
                height_mm=pair["height_m"] * 1000,
                reprojection_rms_px=float(np.sqrt(np.mean(distances**2))),
                corner_p95_px=float(np.percentile(distances, 95)),
                translation_mm=float(np.linalg.norm(predicted_t - observed[:3, 3]) * 1000),
                rotation_deg=float(
                    np.rad2deg(Rotation.from_matrix(predicted_r.T @ observed[:3, :3]).magnitude())
                ),
            )
        )
    return dict(
        metrics={
            key: common.summarize([row[key] for row in rows])
            for key in ("reprojection_rms_px", "corner_p95_px", "translation_mm", "rotation_deg")
        },
        by_height={
            str(h): common.summarize(
                [row["reprojection_rms_px"] for row in rows if row["height_group"] == h]
            )
            for h in sorted({p["height_group"] for p in data["pairs"]})
        },
        by_group={
            str(g): common.summarize(
                [row["reprojection_rms_px"] for row in rows if row["group"] == g]
            )
            for g in sorted({p["group"] for p in data["pairs"]})
        },
        samples=rows,
    )


def representatives(pairs, count):
    output = []
    for group in sorted({p["group"] for p in pairs}):
        selected = sorted([p for p in pairs if p["group"] == group], key=lambda p: p["frame_index"])
        indices = np.unique(
            np.linspace(0, len(selected) - 1, min(count, len(selected))).round().astype(int)
        )
        output += [selected[i] for i in indices]
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=ROOT / "config/robots/calibration/head_camera_j4_rezero_20261104.json",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-nfev", type=int, default=100)
    parser.add_argument("--representatives-per-group", type=int, default=3)
    parser.add_argument("--cv-folds", type=int, default=3)
    parser.add_argument("--models", nargs="*")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=args.resume)
    calibration = json.loads(args.calibration.read_text())
    source = Path(calibration["source"]["pairs"])
    if sha(source) != calibration["source"]["pairs_sha256"]:
        raise ValueError("Calibration source pairs changed")
    test_path = source.parent / "test_pairs.json"
    if sha(test_path) != calibration["validation_summary"]["test_pairs_sha256"]:
        raise ValueError("Frozen test data changed")
    pairs = json.loads(source.read_text())
    test = json.loads(test_path.read_text())
    heldout = {p["group"] for p in test}
    training = [p for p in pairs if p["group"] not in heldout]
    resources = selected_model(ROOT)
    sdk = SDKKinematics(resources["sdk_chains"])
    context = Context(calibration, sdk)
    data = context.data(pairs)
    rotation, translation = context.fk(data["q"])
    cached = np.array([p["A"] for p in pairs])
    fk_error = float(
        max(
            np.max(np.abs(rotation - cached[:, :3, :3])),
            np.max(np.abs(translation - cached[:, :3, 3])),
        )
    )
    if fk_error > 1e-10:
        raise ValueError("Vectorized SDK FK does not reproduce cached measured FK")
    folds = {}
    for h in sorted({p["height_group"] for p in training}):
        groups = sorted({p["group"] for p in training if p["height_group"] == h})
        for i, group in enumerate(groups):
            folds[group] = i % args.cv_folds if args.cv_folds else -1
    provenance = dict(
        source_calibration=str(args.calibration.resolve()),
        source_calibration_sha256=sha(args.calibration),
        pairs=str(source),
        pairs_sha256=sha(source),
        test_pairs_sha256=sha(test_path),
        SDK_chain_sha256=sha(resources["sdk_chains"]),
        URDF_sha256=sha(resources["urdf"]),
        vectorized_FK_max_matrix_error=fk_error,
        training_frames=len(training),
        heldout_frames=len(test),
        training_groups=sorted({p["group"] for p in training}),
        heldout_groups=sorted(heldout),
        representative_frame_ids=[
            p["frame_index"] for p in representatives(training, args.representatives_per_group)
        ],
        cv_fold_by_group=folds,
        max_nfev=args.max_nfev,
        code_sha256=sha(Path(__file__)),
        caveats=[
            "Existing baseline camera/board parameters are reused; inner CV tests incremental corrections, not fully independent prior calibration",
            "Frozen outer holdout has been inspected in previous tasks; fresh poses are required for final acceptance",
            "Only the LEFT arm carries measured board observations; RIGHT joint offsets cannot be inferred from these pairs",
            "No fitted parameter is a hardware zero or deployable control configuration",
        ],
        commands_sent=False,
    )
    if args.resume and (args.output / "provenance.json").exists():
        previous = json.loads((args.output / "provenance.json").read_text())
        for key in (
            "source_calibration_sha256",
            "pairs_sha256",
            "test_pairs_sha256",
            "SDK_chain_sha256",
            "representative_frame_ids",
            "max_nfev",
            "cv_fold_by_group",
            "code_sha256",
            "URDF_sha256",
        ):
            if previous[key] != provenance[key]:
                raise ValueError("Resume input differs: " + key)
    save(args.output / "provenance.json", provenance)
    selected = [spec for spec in catalog() if args.models is None or spec.name in args.models]
    for spec in selected:
        output = args.output / (spec.name + ".json")
        if args.resume and output.exists():
            continue
        print("START", spec.name, flush=True)
        try:
            model = Model(context, spec)
            fit = model.fit(
                context.data(representatives(training, args.representatives_per_group)),
                args.max_nfev,
            )
            result = dict(
                name=spec.name,
                fit=fit,
                model=model.export(),
                training=evaluate(model, context.data(training)),
                heldout=evaluate(model, context.data(test)),
                inner_pose_CV=[],
            )
            for fold in range(args.cv_folds):
                train_fold = [p for p in training if folds[p["group"]] != fold]
                test_fold = [p for p in training if folds[p["group"]] == fold]
                candidate = Model(context, spec)
                fitted = candidate.fit(
                    context.data(representatives(train_fold, args.representatives_per_group)),
                    args.max_nfev,
                )
                validation = evaluate(candidate, context.data(test_fold))
                result["inner_pose_CV"].append(
                    dict(
                        fold=fold,
                        fit=fitted,
                        metrics=validation["metrics"],
                        groups=sorted({p["group"] for p in test_fold}),
                        frame_count=len(test_fold),
                    )
                )
            if spec.height == 2:
                result["new_height_identifiability"] = (
                    "Three recorded heights fit three polynomial coefficients; with one height held out, the quadratic term is not identified by data alone"
                )
            save(output, result)
            print(
                "DONE",
                spec.name,
                "train",
                result["training"]["metrics"]["reprojection_rms_px"]["rms"],
                "heldout",
                result["heldout"]["metrics"]["reprojection_rms_px"]["rms"],
                "rank",
                fit.get("data_rank"),
                "seconds",
                fit["elapsed_s"],
                flush=True,
            )
        except Exception as exc:
            save(
                output,
                dict(
                    name=spec.name,
                    error=str(exc),
                    exception=type(exc).__name__,
                    commands_sent=False,
                ),
            )
            print("FAILED", spec.name, type(exc).__name__, str(exc), flush=True)
    if sha(args.calibration) != provenance["source_calibration_sha256"]:
        raise ValueError("Active calibration changed during comparison")


if __name__ == "__main__":
    main()
