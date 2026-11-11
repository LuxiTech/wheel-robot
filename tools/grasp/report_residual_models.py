#!/usr/bin/env python3
"""Build a local, interactive report from completed offline residual experiments."""

import argparse
import html
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from _bootstrap import configure

configure(ROOT)
from grasp import compare_residual_models as common

LABELS = {
    "baseline_original": "原标定高度（新去畸变）",
    "baseline_URDF_height": "URDF 高度（新去畸变）",
    "image_translation_constant": "整幅图固定像素平移（仅显示）",
    "height_constant": "相机高度：常数修正",
    "height_linear": "相机高度：线性修正",
    "height_quadratic": "相机高度：二次修正",
    "global_translation": "整体三维平移",
    "global_rigid": "整体三维位置＋朝向",
    "camera_and_board_mount": "相机＋棋盘安装关系",
    "left_joint_offsets": "左臂七关节：各自常数",
    "left_joint_affine": "左臂七关节：各自线性",
    "left_joint_nonlinear_sine": "左臂七关节：各自非线性 sin",
    "camera_and_joint_offsets": "相机＋左臂七关节常数",
    "camera_board_and_joint_offsets": "相机＋棋盘安装＋七关节常数",
    "pose_residual_linear_ridge": "整体腕端残差：线性岭回归",
    "pose_residual_quadratic_ridge": "整体腕端残差：二次岭回归",
    "pose_residual_RBF": "整体腕端残差：非线性 RBF",
    "pose_residual_RBF_smoothed": "整体腕端残差：平滑 RBF",
}


def label(name):
    if name.startswith("contour_global_"):
        return "夹指拟合／" + label(name.removeprefix("contour_global_"))
    if name.startswith("left_J") and name.endswith("_offset"):
        return "左臂 " + name.split("_")[1] + " 单独常数"
    if "contour" in name:
        side = "左臂" if name.startswith("left") else "右臂"
        if "_J" in name:
            return side + " " + name.split("_")[1] + " 夹指轮廓拟合"
        kind = {
            "offset": "七关节常数",
            "affine": "七关节线性",
            "sine": "七关节非线性 sin",
            "mount": "腕端到夹爪的固定安装修正",
        }[name.rsplit("_", 1)[1]]
        return side + "／夹指轮廓拟合／" + kind
    return LABELS.get(name, name)


def cv_rms(result):
    rows = result["inner_pose_CV"]
    return float(
        np.sqrt(
            sum(
                row["frame_count"] * row["metrics"]["reprojection_rms_px"]["rms"] ** 2
                for row in rows
            )
            / sum(row["frame_count"] for row in rows)
        )
    )


def table(headers, rows):
    return (
        "<table><thead><tr>"
        + "".join("<th>" + html.escape(str(h)) + "</th>" for h in headers)
        + "</tr></thead><tbody>"
        + "".join(
            "<tr>" + "".join("<td>" + html.escape(str(cell)) + "</td>" for cell in row) + "</tr>"
            for row in rows
        )
        + "</tbody></table>"
    )


def score_rows(rows, split="test", prefix=None):
    selected = [
        row["metrics"]["nominal"]
        for row in rows
        if row["split"] == split and (prefix is None or row["name"].startswith(prefix))
    ]
    if not selected:
        return None
    invalid = any(row["symmetric_boundary_p95_px"] is None for row in selected)
    return dict(
        mean_IoU=float(np.mean([row["silhouette_IoU"] for row in selected])),
        mean_boundary_p95_px=None
        if invalid
        else float(np.mean([row["symmetric_boundary_p95_px"] for row in selected])),
        worst_boundary_p95_px=None
        if invalid
        else float(max(row["symmetric_boundary_p95_px"] for row in selected)),
        outside_image_count=sum(row["symmetric_boundary_p95_px"] is None for row in selected),
        count=len(selected),
    )


def pixel_text(value):
    return "出视野" if value is None else f"{value:.2f}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    fits = root / "offline-v2"
    validation = root / "validation-final"
    provenance = json.loads((fits / "provenance.json").read_text())
    board = [json.loads((fits / (spec.name + ".json")).read_text()) for spec in common.catalog()]
    board.sort(key=cv_rms)
    board_validation = json.loads((validation / "board_fits_gripper_validation.json").read_text())
    contour = sum(
        [
            json.loads((validation / filename).read_text())
            for filename in (
                "global_contour_fits.json",
                "left_contour_fits.json",
                "right_contour_fits.json",
            )
        ],
        [],
    )
    all_overlays = board_validation + contour
    heights = json.loads((validation / "height_holdouts.json").read_text())
    coverage = json.loads((validation / "coverage.json").read_text())
    completed = json.loads((root / "new-poses-retry3/completed.json").read_text())
    board_rows = []
    summary = dict(
        board=[], silhouettes=[], hardware_collection=completed, coverage=coverage, deployable=False
    )
    for result in board:
        fit = result["fit"]
        note = []
        if fit.get("rank_deficient"):
            note.append("参数不能唯一分开")
        if fit.get("at_bounds"):
            note.append("有参数碰搜索边界")
        if result["name"].startswith("pose_residual"):
            note.append("经验函数，不能当硬件零位")
        if not fit["success"]:
            note.append("优化未收敛")
        train = result["training"]["metrics"]["reprojection_rms_px"]["rms"]
        held = result["heldout"]["metrics"]["reprojection_rms_px"]["rms"]
        cv = cv_rms(result)
        board_rows.append(
            [label(result["name"]), f"{train:.3f}", f"{cv:.3f}", f"{held:.3f}", "；".join(note)]
        )
        summary["board"].append(
            dict(
                name=result["name"],
                train_rms_px=train,
                pose_CV_rms_px=cv,
                heldout_rms_px=held,
                note=note,
            )
        )
    figure, axis = plt.subplots(figsize=(11, 5), constrained_layout=True)
    chosen = [
        "baseline_original",
        "height_constant",
        "height_linear",
        "camera_and_board_mount",
        "left_joint_offsets",
        "left_joint_affine",
        "left_joint_nonlinear_sine",
        "camera_board_and_joint_offsets",
        "pose_residual_quadratic_ridge",
        "pose_residual_RBF",
    ]
    lookup = {row["name"]: row for row in summary["board"]}
    x = np.arange(len(chosen))
    for offset, metric, text in [
        (-0.25, "train_rms_px", "Training"),
        (0, "pose_CV_rms_px", "Pose CV"),
        (0.25, "heldout_rms_px", "Frozen holdout"),
    ]:
        axis.bar(x + offset, [lookup[name][metric] for name in chosen], width=0.24, label=text)
    axis.set_xticks(x, [name.replace("_", "\n") for name in chosen], fontsize=8)
    axis.set_ylabel("Raw checkerboard reprojection RMS (pixels)")
    axis.legend()
    axis.grid(axis="y", alpha=0.2)
    figure.savefig(root / "generalization.png", dpi=160)
    plt.close(figure)
    silhouette_rows = []
    for result in all_overlays:
        score = score_rows(result["rows"])
        local = score_rows(result["rows"], prefix="new_")
        unseen_height = score_rows(result["rows"], prefix="historical_minus1")
        summary["silhouettes"].append(
            dict(
                name=result["name"],
                test=score,
                new_local_poses=local,
                withheld_height=unseen_height,
            )
        )
        fit = result.get("fit", {})
        caution = ""
        if fit.get("data_rank", 0) < fit.get("parameter_count", 0):
            caution = "参数不唯一"
        if fit.get("at_bounds"):
            caution += "／碰边界"
        silhouette_rows.append(
            [
                label(result["name"]),
                f"{score['mean_IoU']:.3f}",
                pixel_text(local["mean_boundary_p95_px"]) if local else "—",
                pixel_text(unseen_height["mean_boundary_p95_px"]),
                pixel_text(score["worst_boundary_p95_px"]),
                caution,
            ]
        )
    height_table = []
    for name in (
        "baseline_original",
        "height_constant",
        "height_linear",
        "height_quadratic",
        "camera_and_board_mount",
        "left_joint_offsets",
        "left_joint_affine",
        "left_joint_nonlinear_sine",
        "camera_board_and_joint_offsets",
        "pose_residual_RBF",
        "pose_residual_quadratic_ridge",
    ):
        scores = [row for row in heights if row["name"] == name]
        height_table.append(
            [label(name)]
            + [
                f"{row['metrics']['reprojection_rms_px']['rms']:.3f}"
                if "metrics" in row
                else "不能辨识"
                for row in scores
            ]
        )
    joint_rows = []
    for joint in range(7):
        result = next(row for row in board if row["name"] == f"left_J{joint + 1}_offset")
        cells = [
            f"J{joint + 1}",
            f"{np.rad2deg(result['model']['parameters'][0]):+.3f}°",
            f"{cv_rms(result):.3f}",
        ]
        for side in ("left", "right"):
            fitted = next(
                row for row in contour if row["name"] == f"{side}_J{joint + 1}_contour_offset"
            )
            cells.append(f"{np.rad2deg(fitted['model']['parameters'][0]):+.3f}°")
        joint_rows.append(cells)
    ui_models = []
    for entry in all_overlays:
        rows = []
        for row in entry["rows"]:
            copied = {key: row[key] for key in ("name", "side", "split", "height_um", "metrics")}
            copied["pose"] = row["name"].rsplit("_", 1)[0]
            copied["image"] = str(Path(row["image"]).resolve().relative_to(root))
            copied["original"] = "validation-final/segmentation/" + copied["pose"] + "/photo.png"
            rows.append(copied)
        ui_models.append(dict(name=entry["name"], label=label(entry["name"]), rows=rows))
    data_json = json.dumps(ui_models, ensure_ascii=False).replace("</", "<\\/")
    parts = [
        """<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>残差拟合与跨姿态验证</title><style>
body{font:17px/1.65 system-ui,sans-serif;margin:24px auto;max-width:1450px;padding:0 20px;color:#203246}h1,h2{line-height:1.3}table{border-collapse:collapse;width:100%;margin:14px 0;font-size:15px}td,th{padding:8px;border:1px solid #d7dfe7;text-align:left}thead{background:#edf3f9}tr:nth-child(even){background:#f8fafc}.box{background:#fff5df;padding:18px;border-left:5px solid #d99c27}.controls{background:#eef5fb;padding:14px;position:sticky;top:0;z-index:2}select,button{font-size:16px;padding:8px;max-width:100%;margin:4px}.columns{display:grid;grid-template-columns:1fr 1fr;gap:20px}img{width:100%;height:auto;border:1px solid #d5dce4}code{font-size:14px;overflow-wrap:anywhere}details{padding:10px;border:1px solid #d8e2ec;margin:10px 0}.small{font-size:14px;color:#506276}.metric{min-height:2.5em}a{color:#1260a1}@media(max-width:850px){.columns{grid-template-columns:1fr}}</style>
<h1>残差能拟合，但还没有跨姿态完美重合的模型</h1>
<p class="box">本次先比较 25 种棋盘格模型，再直接检验夹爪轮廓，并比较整体轮廓修正及左右臂各关节常数、线性、非线性模型。复杂模型把训练图片拟合得更好，不代表换姿态后更准。所有修正只用于本报告，没有写入真机零点、URDF 或正式外参。</p>
<p>已有资料：928 帧、21 组左臂棋盘姿态、3 个升降高度；新增：15 组双臂小幅姿态配对图，升降保持约 167.206 mm，另有重启后两种腕部姿态。训练使用新增起始姿态及正向变化；反向变化留出，历史 −1.430 mm 高度也不参与夹指拟合。其余两张历史图用于夹指训练。</p>
<h2>直接看夹指叠图</h2><p>橙色＝对应候选的模型轮廓；绿色＝同帧红色实物区域的颜色分割轮廓。两条线应靠近，但颜色、遮挡和塑料件外观会影响这项指标。只检验夹指，不能当作整条机械臂或钩尖三维坐标已验收。</p>
<div class="controls"><label>照片／姿态 <select id="pose"></select></label><button id="original">切换原图／叠图</button><span id="capture"></span></div>
<div class="columns"><section><h3>机器人左夹爪</h3><select id="left"></select><p class="metric" id="leftMetric"></p><img id="leftImage"></section><section><h3>机器人右夹爪</h3><select id="right"></select><p class="metric" id="rightMetric"></p><img id="rightImage"></section></div>
<h2>整体拟合，还是每个关节都拟合？</h2><p>当前证据支持先检查共同坐标关系和安装关系。按姿态交叉验证，相机＋棋盘安装模型约 3.55 px，优于给七个关节都加非线性修正；但它也没有让真实夹指重合。相机、关节、安装参数能相互吸收误差，拟合出的角度不能直接解释成真机零位偏差。</p>
<h2>棋盘格：训练和换姿态后的差别</h2><p>单位 px，均为原始图像上的 RealSense SDK 精确畸变重投影 RMS。742 帧训练、186 帧姿态整组留出；训练中再做三折姿态交叉验证。优化每组取最多 3 帧并使组权重相同，评估使用全部帧，因此与先前按全部帧加权的单一高度拟合数值略有区别。</p>
<p class="small">这些是新增修正的验证：原相机与棋盘安装参数复用过既有训练数据；历史留出也已在此前任务查看过。不能宣称整个标定流程完全独立。新照片用于额外检验，并未给棋盘格模型重拟合。</p>""",
        table(["模型", "训练 RMS", "姿态交叉验证 RMS", "历史姿态留出 RMS", "限制"], board_rows),
        '<img src="generalization.png" alt="训练与姿态泛化误差比较">',
        "<h2>另一种高度：只用其他两个高度拟合新增修正</h2><p>每列分别留出该高度的训练组；原标定先验仍复用，故这里只检验新增修正的跨高度表现。二次高度函数在仅剩两个高度时没有唯一解，明确不将任意解冒充验证。</p>",
        table(
            ["模型", "171.675 mm 留出 RMS", "81.814 mm 留出 RMS", "−39.990 mm 留出 RMS"],
            height_table,
        ),
        "<h2>夹指实物轮廓：真正换图后的表现</h2><p>IoU 越接近 1 越好，轮廓 P95 越小越好。新反向姿态列检验局部泛化；−1.430 mm 列检验未参与夹指拟合的高度；最差列还包含新拍的大腕转角图。棋盘模型与夹指模型的训练目标不同，不能把两张表的数字当同一种精度。</p>",
        table(
            [
                "模型",
                "全部留出平均 IoU",
                "新反向姿态平均 P95 px",
                "留出高度平均 P95 px",
                "最差图 P95 px",
                "限制",
            ],
            silhouette_rows,
        ),
        "<h2>逐关节单独拟合结果：仅为离线假设</h2><p>每一格均指只改这个关节时的最优候选角度，不能把七个单独最优值相加，也不是机械限位测量的零位偏差。左右臂各自使用自己的照片拟合，没有把左臂数值复制给右臂。</p>",
        table(
            ["关节", "左臂棋盘候选", "棋盘姿态 CV RMS px", "左臂夹指候选", "右臂夹指候选"],
            joint_rows,
        ),
        "<h2>为什么还不能认定通用修正已找到</h2><p>已有左臂棋盘记录的 J5 角度范围与当前抓取姿态不同，非线性函数在范围外会明显失效。夹指颜色轮廓也不能唯一分开相机误差、关节零位和夹爪安装误差。更好的训练重合只说明函数有足够自由度；需要左右腕端、夹爪上独立可识别的同名三维点，在更多姿态和高度下共同验证，才能确定应修正哪一段。</p>",
        "<p>本轮补采完成后已回起始姿态。通信过期、静止采集拒绝、原始状态、请求、停止与恢复后的反馈均保留在采样目录；没有通过放宽运动反馈门限完成采样。</p>",
        '<details><summary>输入、数值与复算记录</summary><p><a href="summary.json">汇总 JSON</a> · <a href="offline-v2/provenance.json">输入指纹与训练划分</a> · <a href="validation-final/coverage.json">关节覆盖范围</a> · <a href="captures-final.json">照片配对清单</a> · <a href="new-poses-retry3/plan.json">采样轨迹</a> · <a href="new-poses-retry3/completed.json">返回姿态记录</a></p><p>源外参 SHA256：<code>'
        + provenance["source_calibration_sha256"]
        + "</code>。报告只诊断；没有选定可部署修正。</p></details>",
        "<script>const models="
        + data_json
        + ";"
        + """
const pose=document.getElementById('pose');let raw=false;
const poses=[...new Set(models[0].rows.map(r=>r.pose))];
for(const p of poses){const o=new Option(p,p);pose.add(o)}
pose.value=poses.includes('new_J7_minus')?'new_J7_minus':poses.at(-1);
for(const side of ['left','right']){const s=document.getElementById(side);for(const m of models){if(m.rows.some(r=>r.side===side))s.add(new Option(m.label,m.name))}s.value='baseline_URDF_height';s.onchange=draw}
function draw(){for(const side of ['left','right']){const model=models.find(m=>m.name===document.getElementById(side).value);const r=model.rows.find(r=>r.side===side&&r.pose===pose.value);const im=document.getElementById(side+'Image');if(!r){im.removeAttribute('src');document.getElementById(side+'Metric').textContent='该模型没有此侧样本';continue}const s=r.metrics.nominal;document.getElementById(side+'Metric').textContent=`${r.split==='train'?'训练图':'留出图'} · 升降 ${(r.height_um/1000).toFixed(3)} mm · IoU ${s.silhouette_IoU.toFixed(3)} · 轮廓P95 ${s.symmetric_boundary_p95_px===null?"出视野":s.symmetric_boundary_p95_px.toFixed(2)+" px"}`;im.src=raw?r.original:r.image}document.getElementById('capture').textContent=raw?'当前显示原图':'当前显示模型与实物轮廓';document.documentElement.dataset.ready='true'}
pose.onchange=draw;document.getElementById('original').onclick=()=>{raw=!raw;draw()};draw();
</script></html>""",
    ]
    (root / "report.html").write_text("\n".join(parts))
    common.save(root / "summary.json", summary)
    print(root / "report.html")


if __name__ == "__main__":
    main()
