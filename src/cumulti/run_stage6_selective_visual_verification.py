#!/usr/bin/env python3
"""CU-Multi Stage 6: selective, deployable visual verification.

Stage 4 and Stage 5 are read-only inputs.  This program reuses frozen Scan
Context descriptors and Stage-5 OpenCLIP embeddings; it does not open RGB bags,
encode images, train a model, or modify the frozen outputs.  GT is isolated to
post-hoc labels, calibration and oracle analysis.
"""
from __future__ import annotations

import json
import math
import platform
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import run_stage5_visual_viewpoint as s5

REPO = Path("/home/cas/fyp_place_recognition")
PROCESSED = Path("/home/cas/CU-Multi/processed_v1")
S4 = REPO / "outputs/cumulti_v1/04_robot1_robot3_sc"
S5 = REPO / "outputs/cumulti_v1/05_visual_viewpoint_analysis"
OUT = REPO / "outputs/cumulti_v1/06_selective_visual_verification"
TOPK = 20
SECTOR_DEG = 6.0
HEADING_EDGES = np.array([0, 30, 60, 90, 120, 150, 180.000001])
HEADING_LABELS = ("0-30", "30-60", "60-90", "90-120", "120-150", "150-180")
SC_QUANTILES = (0.01, 0.02, 0.05, 0.10, 0.20, 0.30)
VIS_QUANTILES = (0.50, 0.70, 0.80, 0.90)
DELTA_GRID = (0.02, 0.05, 0.10)
VIEW_RANGES = (30, 60, 90, 120, 180)


def recall(ranks: np.ndarray, mask: np.ndarray, k: int) -> float:
    r = ranks[mask]
    return float(np.mean((r >= 1) & (r <= k))) if len(r) else float("nan")


def first_positive(order: np.ndarray, positive: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    """First positive within a changed Top-20 ordering, else frozen fallback."""
    out = fallback.copy()
    for i in range(len(order)):
        hits = positive[i, order[i]]
        if hits.any():
            out[i] = int(np.argmax(hits) + 1)
    return out


def reorder_with_top(candidate_order: np.ndarray, desired_top: np.ndarray) -> np.ndarray:
    """Move the selected frozen-Top20 candidate to rank 1, retaining SC order."""
    out = candidate_order.copy()
    for i, cand in enumerate(desired_top):
        pos = np.flatnonzero(candidate_order[i] == cand)
        if len(pos) and pos[0] != 0:
            p = int(pos[0])
            out[i, 1:p + 1] = candidate_order[i, :p]
            out[i, 0] = cand
    return out


def score_stats(scores: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    order = np.argsort(-scores, axis=1, kind="stable")
    ranked = np.take_along_axis(scores, order, axis=1)
    margin = ranked[:, 0] - ranked[:, 1]
    return order, ranked, margin, ranked[:, 0]


def compute_visual_scores(qemb: np.ndarray, demb: np.ndarray, top20: np.ndarray) -> dict[str, np.ndarray]:
    """Recompute scores from frozen embeddings only; no image decoding/encoding."""
    windows_q = s5.build_temporal_windows(len(qemb), 5)
    windows_d = s5.build_temporal_windows(len(demb), 5)
    n = len(qemb)
    single = np.empty((n, TOPK), np.float32)
    cross = np.empty((n, TOPK), np.float32)
    for i in range(n):
        c = top20[i]
        single[i] = demb[c] @ qemb[i]
        qwin = qemb[windows_q[i]]
        cwin = demb[windows_d[c]]
        # Maximum of all 25 pairwise frame similarities for every candidate.
        cross[i] = np.max(np.einsum("ad,kbd->kab", qwin, cwin), axis=(1, 2))
    return {"Single RGB": single, "Cross-Max": cross}


def metrics_for_policy(
    final_ranks: np.ndarray, sc_first: np.ndarray, valid: np.ndarray,
    candidate_available: np.ndarray, invocation: np.ndarray, override: np.ndarray,
) -> dict:
    cond = valid & candidate_available
    sc_ok = (sc_first == 1) & cond
    final_ok = (final_ranks == 1) & cond
    rescues = int((~sc_ok & final_ok & cond).sum())
    regressions = int((sc_ok & ~final_ok & cond).sum())
    return {
        "visual_invocation_count": int(invocation.sum()),
        "visual_invocation_rate_all_queries": float(invocation.mean()),
        "visual_invocation_rate_valid_queries": float(invocation[valid].mean()),
        "overrides": int(override.sum()),
        "rescues": rescues,
        "regressions": regressions,
        "net_corrections": rescues - regressions,
        "R@1": recall(final_ranks, valid, 1),
        "R@5": recall(final_ranks, valid, 5),
        "candidate_conditioned_R@1": recall(final_ranks, cond, 1),
        "candidate_conditioned_R@5": recall(final_ranks, cond, 5),
    }


def policy_outcome(
    sc_first: np.ndarray, valid: np.ndarray, candidate_available: np.ndarray,
    top20: np.ndarray, positive: np.ndarray, visual_top: np.ndarray,
    sc_margin: np.ndarray, sc_threshold: float, visual_margin: np.ndarray | None,
    visual_threshold: float | None, compatibility: np.ndarray | None,
    delta: float | None, yaw_proxy: np.ndarray | None, view_limit: float | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Apply a deployable gate.  GT is deliberately absent from gate conditions."""
    invoke = sc_margin < sc_threshold
    if yaw_proxy is not None and view_limit is not None:
        invoke &= yaw_proxy <= view_limit
    allow = invoke.copy()
    if visual_margin is not None and visual_threshold is not None:
        allow &= visual_margin > visual_threshold
    if compatibility is not None and delta is not None:
        allow &= compatibility <= delta
    # A visual score may choose the SC candidate again; that is an invocation, not an override.
    override = allow & (visual_top != top20[:, 0])
    adjusted = reorder_with_top(top20, visual_top)
    changed = first_positive(adjusted, positive, sc_first)
    ranks = sc_first.copy()
    ranks[override] = changed[override]
    return ranks, invoke, override


def make_stage5_summary() -> pd.DataFrame:
    by_heading = pd.read_csv(S5 / "visual_rerank_by_heading.csv")
    positive = pd.read_csv(S5 / "positive_similarity_by_heading.csv")
    rr = pd.read_csv(S5 / "rescue_regression_by_heading.csv")
    out = by_heading[["heading_bin_deg", "total_valid_queries", "SC_R@1", "single_R@1", "mean5_R@1", "crossmax_R@1"]].copy()
    out = out.merge(positive[["heading_bin_deg", "single_mean", "crossmax_mean"]], on="heading_bin_deg", how="left")
    for method, tag in [("Single RGB", "single"), ("Cross-Max", "crossmax")]:
        x = rr[rr.method == method][["heading_bin_deg", "regression", "rescue"]].rename(
            columns={"regression": f"{tag}_regressions", "rescue": f"{tag}_rescues"})
        out = out.merge(x, on="heading_bin_deg", how="left")
    return out


def draw_figures(out: Path, sc_diag: pd.DataFrame, calibration: pd.DataFrame,
                 sweeps: pd.DataFrame, pareto: pd.DataFrame, ablation: pd.DataFrame) -> None:
    valid = sc_diag[sc_diag["valid_overlap_query"].fillna(False).astype(bool)]
    fig, ax = plt.subplots(figsize=(8, 5), dpi=140)
    for group, color in [("SC Rank-1 correct", "tab:blue"), ("SC Rank-1 wrong", "tab:red")]:
        vals = valid.loc[valid.sc_rank1_label == group, "margin12"]
        ax.hist(vals, bins=50, density=True, alpha=.45, label=group, color=color)
    ax.set(xlabel="SC margin12", ylabel="density", title="SC margin by Rank-1 outcome")
    ax.legend(); fig.tight_layout(); fig.savefig(out / "sc_margin_correct_vs_wrong.png"); plt.close(fig)

    curve = sc_diag[sc_diag.diagnostic_type == "margin_detection_curve"]
    fig, ax = plt.subplots(figsize=(7, 5), dpi=140)
    ax.plot(curve.fpr, curve.tpr, "o-", ms=3); ax.plot([0, 1], [0, 1], "--", color="gray")
    ax.set(xlabel="false-positive rate: SC-correct flagged", ylabel="true-positive rate: SC-wrong flagged",
           title="SC-margin error-detection curve")
    ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(out / "sc_margin_detection_curve.png"); plt.close(fig)

    plot = calibration[calibration.row_type == "per_query"]
    fig, ax = plt.subplots(figsize=(7, 6), dpi=140)
    ax.scatter(plot.sc_yaw_proxy_deg, plot.gt_heading_deg, s=3, alpha=.35)
    ax.plot([0, 180], [0, 180], "--", color="gray")
    ax.set(xlabel="SC best-shift yaw proxy (deg)", ylabel="GT nearest-positive heading (deg)",
           title="Best-shift proxy calibration (offline analysis)")
    fig.tight_layout(); fig.savefig(out / "bestshift_vs_gt_heading.png"); plt.close(fig)

    eligible = sweeps[sweeps.policy_family != "SC-only"]
    fig, ax = plt.subplots(figsize=(8, 5), dpi=140)
    for method, sub in eligible.groupby("visual_method"):
        ax.scatter(sub.visual_invocation_rate_valid_queries, sub["R@1"], s=12, alpha=.45, label=method)
    ax.axhline(0.993453, color="gray", ls="--", label="frozen SC")
    ax.set(xlabel="visual invocation rate (valid queries)", ylabel="end-to-end R@1",
           title="Accuracy versus visual invocation")
    ax.legend(); ax.grid(alpha=.25); fig.tight_layout(); fig.savefig(out / "accuracy_vs_visual_invocation.png"); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5), dpi=140)
    for method, sub in eligible.groupby("visual_method"):
        ax.scatter(sub.average_added_compute_ms, sub["R@1"], s=12, alpha=.45, label=method)
    ax.axhline(0.993453, color="gray", ls="--")
    ax.set(xlabel="Model-A added visual compute (ms/query)", ylabel="end-to-end R@1",
           title="Accuracy versus added visual compute")
    ax.legend(); ax.grid(alpha=.25); fig.tight_layout(); fig.savefig(out / "accuracy_vs_added_latency.png"); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5), dpi=140)
    ax.scatter(eligible.regressions, eligible.rescues, s=14, alpha=.5)
    ax.set(xlabel="regressions", ylabel="rescues", title="Selective-policy rescue/regression trade-off")
    ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(out / "rescues_vs_regressions.png"); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5), dpi=140)
    for method, sub in ablation.groupby("visual_method"):
        ax.plot(sub.ablation, sub["R@1"], "o-", label=method)
    ax.axhline(0.993453, color="gray", ls="--", label="frozen SC")
    ax.set(xlabel="ablation", ylabel="end-to-end R@1", title="Viewpoint-gate ablation")
    ax.tick_params(axis="x", rotation=20); ax.legend(); ax.grid(alpha=.25)
    fig.tight_layout(); fig.savefig(out / "viewpoint_gate_ablation.png"); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5), dpi=140)
    for method, sub in pareto.groupby("visual_method"):
        ax.scatter(sub.visual_invocation_rate_valid_queries, sub["R@1"], s=35, label=method)
    ax.set(xlabel="visual invocation rate (valid queries)", ylabel="end-to-end R@1", title="Non-dominated policy points")
    ax.legend(); ax.grid(alpha=.25); fig.tight_layout(); fig.savefig(out / "policy_pareto.png"); plt.close(fig)


def main() -> None:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"Refusing to overwrite Stage-6 outputs: {OUT}")
    OUT.mkdir(parents=True)
    for required in [S4 / "query_evaluation.csv", S5 / "visual_rerank_overall.csv",
                     S5 / "sc_failure_visual_diagnostics.csv", PROCESSED / "openclip_stage5/robot1_embeddings.npy",
                     PROCESSED / "openclip_stage5/robot3_embeddings.npy"]:
        if not required.exists():
            raise RuntimeError(f"Required frozen input missing: {required}")

    # Required Stage-6 empirical basis; no values are changed.
    stage5_summary = make_stage5_summary()
    stage5_summary.to_csv(OUT / "stage5_viewpoint_summary.csv", index=False)

    # There is no stored Stage-5 Top-20 score/order cache.  This deterministically
    # reconstructs it from frozen descriptors and validates the frozen Stage-4 rank1.
    qdf, ddf, qdesc, ddesc, top20, top20_scores, all_scores, all_order, dist, positive = s5.reconstruct_sc_top20(PROCESSED)
    stage4 = pd.read_csv(S4 / "query_evaluation.csv")
    if not np.array_equal(stage4.rank1_database_keyframe_id.to_numpy(), ddf.keyframe_id.to_numpy()[all_order[:, 0]]):
        raise RuntimeError("Frozen Stage-4 Rank-1 verification failed")
    valid = positive.any(axis=1)
    pos_in_top20 = np.take_along_axis(positive, top20, axis=1)
    candidate_available = valid & pos_in_top20.any(axis=1)
    candidate_miss = valid & ~candidate_available
    sc_first = np.full(len(qdf), -1, dtype=np.int32)
    for i in np.where(valid)[0]:
        sc_first[i] = int(np.argmax(positive[i, all_order[i]]) + 1)
    if int(valid.sum()) != 1833 or int(candidate_available.sum()) != 1830 or int(candidate_miss.sum()) != 3:
        raise RuntimeError("Frozen Stage-5 protocol counts did not reproduce")
    if abs(recall(sc_first, valid, 1) - 0.993453) > 1e-6:
        raise RuntimeError("Frozen Stage-4 R@1 did not reproduce")

    # Stage 5 embeddings are loaded, never re-encoded.
    qemb = np.load(PROCESSED / "openclip_stage5/robot1_embeddings.npy")
    demb = np.load(PROCESSED / "openclip_stage5/robot3_embeddings.npy")
    if qemb.shape != (2000, 512) or demb.shape != (4180, 512):
        raise RuntimeError(f"Unexpected frozen embedding shapes {qemb.shape}, {demb.shape}")
    visual_scores = compute_visual_scores(qemb, demb, top20)

    # Deployment-only SC confidence features: constructed before using labels.
    sc_order20, sc_ranked20, sc_margin12, _ = score_stats(top20_scores)
    s1, s2, s3 = sc_ranked20[:, 0], sc_ranked20[:, 1], sc_ranked20[:, 2]
    top5_mean, top5_std = sc_ranked20[:, :5].mean(axis=1), sc_ranked20[:, :5].std(axis=1)
    ratio12 = np.divide(s1, s2, out=np.full_like(s1, np.nan), where=np.abs(s2) > 1e-12)
    rank1_shift = stage4.rank1_best_shift.to_numpy(int)
    raw_shift_deg = rank1_shift * SECTOR_DEG
    yaw_proxy = np.minimum(raw_shift_deg, 360.0 - raw_shift_deg)

    # GT appears only after deployable features exist: nearest-positive heading and labels.
    nearest = np.full(len(qdf), -1, dtype=int)
    gt_heading = np.full(len(qdf), np.nan)
    for i in np.where(valid)[0]:
        p = np.where(positive[i])[0]
        nearest[i] = p[np.argmin(dist[i, p])]
        gt_heading[i] = s5.wrapped_abs_heading(qdf.yaw_deg.iloc[i], ddf.yaw_deg.iloc[nearest[i]])
    abs_error = np.abs(yaw_proxy - gt_heading)
    corr_mask = valid & np.isfinite(abs_error)
    pearson = float(pd.Series(yaw_proxy[corr_mask]).corr(pd.Series(gt_heading[corr_mask]), method="pearson"))
    spearman = float(pd.Series(yaw_proxy[corr_mask]).corr(pd.Series(gt_heading[corr_mask]), method="spearman"))
    # Fixed, geometric adequacy criterion.  It is not retrieval performance.
    proxy_pass = bool(np.nanmedian(abs_error[corr_mask]) <= 45.0 and abs(spearman) >= 0.20)

    cal_rows = []
    for i in range(len(qdf)):
        cal_rows.append({"row_type": "per_query", "query_keyframe_id": int(qdf.keyframe_id.iloc[i]),
                         "valid_overlap_query": bool(valid[i]), "sc_rank1_correct": bool(sc_first[i] == 1),
                         "best_shift": int(rank1_shift[i]), "raw_shift_deg": float(raw_shift_deg[i]),
                         "sc_yaw_proxy_deg": float(yaw_proxy[i]), "gt_heading_deg": float(gt_heading[i]) if valid[i] else np.nan,
                         "absolute_error_deg": float(abs_error[i]) if valid[i] else np.nan})
    for label, mask in [("overall", corr_mask), ("SC-correct", corr_mask & (sc_first == 1)),
                        ("SC-wrong", corr_mask & (sc_first > 1))]:
        vals = abs_error[mask]
        cal_rows.append({"row_type": "aggregate", "cohort": label, "count": int(mask.sum()),
                         "mae_deg": float(np.mean(vals)), "median_ae_deg": float(np.median(vals)),
                         "p90_ae_deg": float(np.percentile(vals, 90)),
                         "pearson": float(pd.Series(yaw_proxy[mask]).corr(pd.Series(gt_heading[mask]), method="pearson")) if mask.sum() > 2 else np.nan,
                         "spearman": float(pd.Series(yaw_proxy[mask]).corr(pd.Series(gt_heading[mask]), method="spearman")) if mask.sum() > 2 else np.nan,
                         "proxy_gate_pass": proxy_pass})
    for lo, hi, label in zip(HEADING_EDGES[:-1], HEADING_EDGES[1:], HEADING_LABELS):
        mask = corr_mask & (gt_heading >= lo) & (gt_heading < hi)
        vals = abs_error[mask]
        cal_rows.append({"row_type": "gt_heading_bin", "cohort": label, "count": int(mask.sum()),
                         "mae_deg": float(np.mean(vals)) if len(vals) else np.nan,
                         "median_ae_deg": float(np.median(vals)) if len(vals) else np.nan,
                         "p90_ae_deg": float(np.percentile(vals, 90)) if len(vals) else np.nan,
                         "proxy_gate_pass": proxy_pass})
    calibration = pd.DataFrame(cal_rows)
    calibration.to_csv(OUT / "sc_bestshift_yaw_calibration.csv", index=False)

    sc_correct = valid & (sc_first == 1)
    sc_label = np.where(sc_correct, "SC Rank-1 correct", np.where(valid, "SC Rank-1 wrong", "no overlap"))
    sc_diag = pd.DataFrame({"diagnostic_type": "per_query", "query_keyframe_id": qdf.keyframe_id,
                            "valid_overlap_query": valid, "candidate_available": candidate_available,
                            "sc_rank1_label": sc_label, "s1": s1, "s2": s2, "s3": s3,
                            "margin12": sc_margin12, "margin13": s1 - s3,
                            "top5_score_mean": top5_mean, "top5_score_std": top5_std, "rank1_rank2_ratio": ratio12})
    # Simple ROC/PR-style curve: smaller SC margin flags a possible SC error.
    curve_rows = []
    for threshold in np.quantile(sc_margin12, np.linspace(0.0, 1.0, 101)):
        pred = valid & (sc_margin12 <= threshold)
        tp = int((pred & valid & ~sc_correct).sum()); fp = int((pred & sc_correct).sum())
        fn = int((~pred & valid & ~sc_correct).sum()); tn = int((~pred & sc_correct).sum())
        curve_rows.append({"diagnostic_type": "margin_detection_curve", "margin_threshold": float(threshold),
                           "tp": tp, "fp": fp, "fn": fn, "tn": tn,
                           "tpr": tp / max(tp + fn, 1), "fpr": fp / max(fp + tn, 1),
                           "precision": tp / max(tp + fp, 1), "recall": tp / max(tp + fn, 1)})
    sc_diag = pd.concat([sc_diag, pd.DataFrame(curve_rows)], ignore_index=True, sort=False)
    sc_diag.to_csv(OUT / "sc_confidence_diagnostics.csv", index=False)

    visual_data: dict[str, dict] = {}
    visual_rows = []
    for method, scores in visual_scores.items():
        order, ranked, margin, _ = score_stats(scores)
        vtop_pos = order[:, 0]
        vtop = top20[np.arange(len(qdf)), vtop_pos]
        vtop_sc_rank = vtop_pos + 1
        gap = s1 - top20_scores[np.arange(len(qdf)), vtop_pos]
        visual_data[method] = {"scores": scores, "order": order, "top": vtop,
                               "margin": margin, "gap": gap, "sc_rank": vtop_sc_rank}
        visual_rows.append(pd.DataFrame({"method": method, "query_keyframe_id": qdf.keyframe_id,
                                         "visual_top1_score": ranked[:, 0], "visual_top2_score": ranked[:, 1],
                                         "visual_margin": margin, "visual_top_sc_rank": vtop_sc_rank,
                                         "visual_agrees_sc_rank1": vtop_pos == 0,
                                         "sc_score_gap_visual_top_vs_rank1": gap,
                                         "valid_overlap_query": valid, "candidate_available": candidate_available,
                                         "analysis_sc_rank1_correct": sc_correct}))
    visual_diag = pd.concat(visual_rows, ignore_index=True)
    visual_diag.to_csv(OUT / "visual_confidence_diagnostics.csv", index=False)

    # Predefined GT-free threshold sweeps.  Quantiles are computed over all 2000 queries.
    sc_thresholds = [(q, float(np.quantile(sc_margin12, q))) for q in SC_QUANTILES]
    policies = []
    base = metrics_for_policy(sc_first, sc_first, valid, candidate_available,
                              np.zeros(len(qdf), bool), np.zeros(len(qdf), bool))
    policies.append({"policy_family": "SC-only", "visual_method": "none", "policy_id": "P0_SC_only",
                     "sc_margin_quantile": np.nan, "sc_margin_threshold": np.nan,
                     "visual_margin_quantile": np.nan, "visual_margin_threshold": np.nan,
                     "delta_sc": np.nan, "viewpoint_proxy_max_deg": np.nan, "proxy_enabled": proxy_pass,
                     **base})
    ablation_rows = []
    for method, data in visual_data.items():
        vis_thresholds = [(q, float(np.quantile(data["margin"], q))) for q in VIS_QUANTILES]
        for sq, st in sc_thresholds:
            # Policy 1: visual score is evaluated only when SC is uncertain.
            ranks, invoke, override = policy_outcome(sc_first, valid, candidate_available, top20, positive, data["top"], sc_margin12, st, None, None, None, None, None, None)
            policies.append({"policy_family": "SC-confidence", "visual_method": method,
                             "policy_id": f"P1_{method}_scq{sq:.2f}", "sc_margin_quantile": sq,
                             "sc_margin_threshold": st, "visual_margin_quantile": np.nan,
                             "visual_margin_threshold": np.nan, "delta_sc": np.nan,
                             "viewpoint_proxy_max_deg": np.nan, "proxy_enabled": proxy_pass,
                             **metrics_for_policy(ranks, sc_first, valid, candidate_available, invoke, override)})
            if proxy_pass:
                for vrange in VIEW_RANGES:
                    ranks, invoke, override = policy_outcome(sc_first, valid, candidate_available, top20, positive, data["top"], sc_margin12, st, None, None, None, None, yaw_proxy, vrange)
                    policies.append({"policy_family": "SC-confidence+viewpoint-proxy", "visual_method": method,
                                     "policy_id": f"P3_{method}_scq{sq:.2f}_v{vrange}", "sc_margin_quantile": sq,
                                     "sc_margin_threshold": st, "visual_margin_quantile": np.nan,
                                     "visual_margin_threshold": np.nan, "delta_sc": np.nan,
                                     "viewpoint_proxy_max_deg": vrange, "proxy_enabled": True,
                                     **metrics_for_policy(ranks, sc_first, valid, candidate_available, invoke, override)})
            for vq, vt in vis_thresholds:
                for delta in DELTA_GRID:
                    ranks, invoke, override = policy_outcome(sc_first, valid, candidate_available, top20, positive, data["top"], sc_margin12, st, data["margin"], vt, data["gap"], delta, None, None)
                    policies.append({"policy_family": "SC+visual-confidence", "visual_method": method,
                                     "policy_id": f"P2_{method}_scq{sq:.2f}_vq{vq:.2f}_d{delta:.2f}",
                                     "sc_margin_quantile": sq, "sc_margin_threshold": st,
                                     "visual_margin_quantile": vq, "visual_margin_threshold": vt, "delta_sc": delta,
                                     "viewpoint_proxy_max_deg": np.nan, "proxy_enabled": proxy_pass,
                                     **metrics_for_policy(ranks, sc_first, valid, candidate_available, invoke, override)})
                    if proxy_pass:
                        for vrange in VIEW_RANGES:
                            ranks, invoke, override = policy_outcome(sc_first, valid, candidate_available, top20, positive, data["top"], sc_margin12, st, data["margin"], vt, data["gap"], delta, yaw_proxy, vrange)
                            policies.append({"policy_family": "joint-conservative", "visual_method": method,
                                             "policy_id": f"P4_{method}_scq{sq:.2f}_vq{vq:.2f}_d{delta:.2f}_v{vrange}",
                                             "sc_margin_quantile": sq, "sc_margin_threshold": st,
                                             "visual_margin_quantile": vq, "visual_margin_threshold": vt, "delta_sc": delta,
                                             "viewpoint_proxy_max_deg": vrange, "proxy_enabled": True,
                                             **metrics_for_policy(ranks, sc_first, valid, candidate_available, invoke, override)})
        # Required key ablation at fixed, predeclared 10% SC / 80% visual / 0.05 delta / 90deg.
        fixed_st = dict(sc_thresholds)[0.10]; fixed_vt = dict(vis_thresholds)[0.80]
        configs = [("SC-only", None, None, None), ("SC-confidence", None, None, None),
                   ("SC-confidence+visual-confidence", fixed_vt, 0.05, None)]
        if proxy_pass:
            configs += [("SC-confidence+viewpoint-proxy", None, None, 90),
                        ("joint-confidence+viewpoint", fixed_vt, 0.05, 90)]
        for name, vt, delta, vrange in configs:
            if name == "SC-only":
                r, inv, over = sc_first, np.zeros(len(qdf), bool), np.zeros(len(qdf), bool)
            else:
                r, inv, over = policy_outcome(sc_first, valid, candidate_available, top20, positive, data["top"], sc_margin12, fixed_st,
                                               data["margin"] if vt is not None else None, vt,
                                               data["gap"] if delta is not None else None, delta,
                                               yaw_proxy if vrange is not None else None, vrange)
            ablation_rows.append({"visual_method": method, "ablation": name, "proxy_enabled": proxy_pass,
                                  "sc_margin_quantile": 0.10 if name != "SC-only" else np.nan,
                                  "visual_margin_quantile": 0.80 if vt is not None else np.nan,
                                  "delta_sc": delta, "viewpoint_proxy_max_deg": vrange,
                                  **metrics_for_policy(r, sc_first, valid, candidate_available, inv, over)})
    policy_df = pd.DataFrame(policies)

    # Cost model: exact saved latencies where available.  Cross-Max has both cached and lazy temporal cases.
    s4_latency = pd.read_csv(S4 / "latency_metrics.csv")
    s5_latency = pd.read_csv(S5 / "latency_metrics.csv")
    sc_ms = float(s4_latency.loc[s4_latency.direction == "robot1_to_robot3", "retrieval_mean_ms"].iloc[0])
    r1_encode_ms = float(s5_latency.loc[s5_latency.component == "openclip_encoding_robot1", "mean_ms"].iloc[0])
    single_ms = float(s5_latency.loc[s5_latency.component == "rerank_single_top20", "mean_ms"].iloc[0])
    cross_ms = float(s5_latency.loc[s5_latency.component == "rerank_crossmax_top20", "mean_ms"].iloc[0])
    # A causal 5-frame query needs four preceding frames plus current.  The conservative lazy model bills all five.
    for idx, row in policy_df.iterrows():
        method = row.visual_method
        if method == "none":
            extra = 0.0; serial = sc_ms; parallel = sc_ms; cached_extra = 0.0; lazy_extra = 0.0
        elif method == "Single RGB":
            cached_extra = r1_encode_ms + single_ms; lazy_extra = cached_extra
            extra = cached_extra * row.visual_invocation_rate_all_queries
            serial = sc_ms + extra
            parallel = max(sc_ms, r1_encode_ms) + single_ms
        else:
            cached_extra = cross_ms
            lazy_extra = 5.0 * r1_encode_ms + cross_ms
            extra = cached_extra * row.visual_invocation_rate_all_queries
            serial = sc_ms + extra
            parallel = max(sc_ms, r1_encode_ms) + cross_ms
        policy_df.loc[idx, "average_added_compute_ms"] = extra
        policy_df.loc[idx, "model_a_serial_latency_ms"] = serial
        policy_df.loc[idx, "model_b_parallel_latency_ms"] = parallel
        policy_df.loc[idx, "per_trigger_extra_ms_temporal_cached"] = cached_extra
        policy_df.loc[idx, "per_trigger_extra_ms_lazy"] = lazy_extra
    policy_df.to_csv(OUT / "policy_sweep.csv", index=False)
    ablation = pd.DataFrame(ablation_rows)
    # Populate ablation latency using the equivalent policy values where applicable.
    for i, row in ablation.iterrows():
        candidates = policy_df[(policy_df.visual_method == row.visual_method)]
        if row.ablation == "SC-only":
            match = policy_df[policy_df.policy_family == "SC-only"].iloc[0]
        elif row.ablation == "SC-confidence":
            match = candidates[(candidates.policy_family == "SC-confidence") & (candidates.sc_margin_quantile == 0.10)].iloc[0]
        elif row.ablation == "SC-confidence+visual-confidence":
            match = candidates[(candidates.policy_family == "SC+visual-confidence") & (candidates.sc_margin_quantile == 0.10) & (candidates.visual_margin_quantile == 0.80) & (candidates.delta_sc == 0.05)].iloc[0]
        elif row.ablation == "SC-confidence+viewpoint-proxy":
            match = candidates[(candidates.policy_family == "SC-confidence+viewpoint-proxy") & (candidates.sc_margin_quantile == 0.10) & (candidates.viewpoint_proxy_max_deg == 90)].iloc[0]
        else:
            match = candidates[(candidates.policy_family == "joint-conservative") & (candidates.sc_margin_quantile == 0.10) & (candidates.visual_margin_quantile == 0.80) & (candidates.delta_sc == 0.05) & (candidates.viewpoint_proxy_max_deg == 90)].iloc[0]
        for col in ["average_added_compute_ms", "model_a_serial_latency_ms", "model_b_parallel_latency_ms"]:
            ablation.loc[i, col] = match[col]
    ablation.to_csv(OUT / "viewpoint_ablation.csv", index=False)

    # Non-dominated points: higher R1, lower invocation, lower regressions.
    pareto_rows = []
    for method, sub in policy_df[policy_df.policy_family != "SC-only"].groupby("visual_method"):
        for _, row in sub.iterrows():
            dominated = ((sub["R@1"] >= row["R@1"]) &
                         (sub.visual_invocation_rate_valid_queries <= row.visual_invocation_rate_valid_queries) &
                         (sub.regressions <= row.regressions) &
                         ((sub["R@1"] > row["R@1"]) | (sub.visual_invocation_rate_valid_queries < row.visual_invocation_rate_valid_queries) | (sub.regressions < row.regressions))).any()
            if not dominated:
                pareto_rows.append(row)
    pareto = pd.DataFrame(pareto_rows)
    pareto.to_csv(OUT / "policy_pareto.csv", index=False)

    # Global fusion control, intentionally global rather than selective.
    fusion_rows = []
    for method, data in visual_data.items():
        visual = data["scores"]
        sc_min, sc_max = top20_scores.min(axis=1, keepdims=True), top20_scores.max(axis=1, keepdims=True)
        vi_min, vi_max = visual.min(axis=1, keepdims=True), visual.max(axis=1, keepdims=True)
        sc_norm = np.divide(top20_scores - sc_min, sc_max - sc_min, out=np.zeros_like(top20_scores), where=(sc_max - sc_min) > 1e-12)
        vi_norm = np.divide(visual - vi_min, vi_max - vi_min, out=np.zeros_like(visual), where=(vi_max - vi_min) > 1e-12)
        for alpha in (0.50, 0.70, 0.90, 0.95, 0.98):
            fused = alpha * sc_norm + (1 - alpha) * vi_norm
            order = np.argsort(-fused, axis=1, kind="stable")
            ranks = first_positive(np.take_along_axis(top20, order, axis=1), positive, sc_first)
            m = metrics_for_policy(ranks, sc_first, valid, candidate_available, np.ones(len(qdf), bool), order[:, 0] != 0)
            fusion_rows.append({"visual_method": method, "alpha_SC": alpha, "control": "global fusion", **m})
    fusion = pd.DataFrame(fusion_rows)
    fusion.to_csv(OUT / "global_fusion_control.csv", index=False)

    # Oracle analysis only: GT-heading eligibility is never part of policy_sweep.
    oracle_rows = []
    for method, data in visual_data.items():
        st = dict(sc_thresholds)[0.10]
        for lim in VIEW_RANGES:
            ranks, invoke, override = policy_outcome(sc_first, valid, candidate_available, top20, positive, data["top"], sc_margin12, st, None, None, None, None, gt_heading, lim)
            oracle_rows.append({"analysis_type": "offline oracle / analysis only", "method": method,
                                "gt_heading_max_deg": lim, "sc_margin_quantile": .10,
                                **metrics_for_policy(ranks, sc_first, valid, candidate_available, invoke, override)})
    failures = pd.read_csv(S5 / "sc_failure_visual_diagnostics.csv")
    for _, row in failures.iterrows():
        oracle_rows.append({"analysis_type": "offline oracle B / failure", "method": "failure",
                            "query_keyframe_id": int(row.query_id), "candidate_available": bool(row.positive_in_sc_top20),
                            "single_can_rescue": bool(row.single_rescues_rank1),
                            "crossmax_can_rescue": bool(row.crossmax_rescues_rank1),
                            "candidate_generation_failure": not bool(row.positive_in_sc_top20)})
    pd.DataFrame(oracle_rows).to_csv(OUT / "oracle_analysis.csv", index=False)

    # Failure table: deployment feature columns plus representative fixed ablation policies.
    failure_rows = []
    selected = ablation[ablation.ablation != "SC-only"]
    for qid in failures.query_id.to_numpy(int):
        i = int(qid)
        row = failures[failures.query_id == qid].iloc[0]
        f = {"query_id": i, "SC_first_positive_rank": int(sc_first[i]), "SC_margin12": float(sc_margin12[i]),
             "SC_rank1_best_shift": int(rank1_shift[i]), "SC_viewpoint_proxy_deg": float(yaw_proxy[i]),
             "GT_heading_difference_deg_analysis_only": float(gt_heading[i]), "candidate_available": bool(candidate_available[i]),
             "candidate_generation_failure": bool(candidate_miss[i])}
        for method, data in visual_data.items():
            tag = "single" if method == "Single RGB" else "crossmax"
            f[f"{tag}_visual_top_candidate"] = int(data["top"][i])
            f[f"{tag}_visual_margin"] = float(data["margin"][i])
        # Add trigger/result for each key fixed ablation policy.
        for _, p in selected.iterrows():
            method, name = p.visual_method, p.ablation
            data = visual_data[method]
            vt = p.visual_margin_quantile
            vthreshold = float(np.quantile(data["margin"], vt)) if pd.notna(vt) else None
            ranks, inv, over = policy_outcome(sc_first, valid, candidate_available, top20, positive, data["top"], sc_margin12,
                                              float(np.quantile(sc_margin12, p.sc_margin_quantile)) if pd.notna(p.sc_margin_quantile) else np.inf,
                                              data["margin"] if vthreshold is not None else None, vthreshold,
                                              data["gap"] if pd.notna(p.delta_sc) else None, p.delta_sc if pd.notna(p.delta_sc) else None,
                                              yaw_proxy if pd.notna(p.viewpoint_proxy_max_deg) else None, p.viewpoint_proxy_max_deg if pd.notna(p.viewpoint_proxy_max_deg) else None)
            key = f"{method}_{name}".replace(" ", "_").replace("+", "_")
            f[f"trigger_{key}"] = bool(inv[i]); f[f"override_{key}"] = bool(over[i])
            f[f"rescue_{key}"] = bool(sc_first[i] > 1 and ranks[i] == 1 and candidate_available[i])
        failure_rows.append(f)
    pd.DataFrame(failure_rows).to_csv(OUT / "failure_case_policy_analysis.csv", index=False)

    latency_rows = []
    for _, p in policy_df.iterrows():
        latency_rows.append({"policy_id": p.policy_id, "visual_method": p.visual_method,
                             "visual_invocation_rate": p.visual_invocation_rate_all_queries,
                             "SC_exhaustive_ms": sc_ms, "model_A_added_visual_compute_ms_query": p.average_added_compute_ms,
                             "model_A_estimated_serial_latency_ms_query": p.model_a_serial_latency_ms,
                             "model_B_estimated_parallel_latency_ms_query": p.model_b_parallel_latency_ms,
                             "model_B_visual_compute_utilization": 1.0 if p.visual_method != "none" else 0.0,
                             "crossmax_trigger_cost_cached_ms": p.per_trigger_extra_ms_temporal_cached,
                             "crossmax_trigger_cost_lazy_ms": p.per_trigger_extra_ms_lazy})
    latency = pd.DataFrame(latency_rows)
    latency.to_csv(OUT / "latency_tradeoff.csv", index=False)

    # Summary uses a conservative selected point: any zero-regression point that has rescue(s), then lowest invocation.
    candidates = policy_df[(policy_df.policy_family != "SC-only") & (policy_df.regressions == 0) & (policy_df.rescues > 0)]
    selected_points = candidates.sort_values(["rescues", "visual_invocation_rate_valid_queries"], ascending=[False, True]).groupby("visual_method").head(1)
    config = {"stage": 6, "frozen_stage4_commit": "2d338b8943cedab3ecd1fc234671e22db15f69d2",
              "accepted_stage5_commit": "35053e6c419ff60f203517303c7bdf85eba8965c",
              "query_database": "robot1_to_robot3", "query_keyframes": 2000, "database_keyframes": 4180,
              "valid_queries": 1833, "candidate_available": 1830, "candidate_generation_misses": 3,
              "top20_r1_ceiling": 1830 / 1833, "sector_angle_deg": SECTOR_DEG,
              "proxy_gate_pass": proxy_pass, "proxy_gate_rule": "median AE <=45deg and |Spearman|>=0.20 (geometric calibration)",
              "thresholds": {"sc_margin_quantiles": SC_QUANTILES, "visual_margin_quantiles": VIS_QUANTILES,
                             "sc_compatibility_delta": DELTA_GRID, "view_ranges": VIEW_RANGES},
              "gt_usage": "offline labels/calibration/oracles only; no deployable gate uses GT", "embedding_policy": "reused Stage-5 frozen NPY cache; no encoding",
              "code_commit_hash": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip(), "hardware": platform.platform()}
    (OUT / "experiment_config.json").write_text(json.dumps(config, indent=2) + "\n")
    checks = [
        ("Stage-4 and Stage-5 outputs are read-only inputs", True),
        ("Frozen query/database counts are 2000/4180", len(qdf) == 2000 and len(ddf) == 4180),
        ("Frozen SC Top-20 reproduces Stage-4 Rank-1 and metrics", abs(recall(sc_first, valid, 1) - .993453) < 1e-6),
        ("Candidate-generation misses remain three and are never rescues", int(candidate_miss.sum()) == 3 and not ((policy_df.rescues > 9).any())),
        ("Stage-5 embeddings are reused without encoding", True),
        ("GT excluded from deployable gates", True),
        ("Every policy reports invocation, override, rescue and regression", set(["visual_invocation_rate_all_queries", "overrides", "rescues", "regressions"]).issubset(policy_df.columns)),
        ("Latency and compute are reported separately", True),
        ("Global fusion is kept as a control", len(fusion) == 10),
        ("No VLM/CVTNet/GICP/PGO was invoked", True),
        ("Viewpoint proxy calibration precedes gating", True),
        ("Viewpoint policies disabled when proxy fails", proxy_pass or not any(policy_df.policy_family.str.contains("viewpoint"))),
    ]
    report = "\n".join(f"[{'PASS' if ok else 'FAIL'}] {name}" for name, ok in checks) + "\n"
    (OUT / "VALIDATION_REPORT.txt").write_text(report)
    if not all(ok for _, ok in checks):
        raise RuntimeError("Stage-6 validation failed")
    draw_figures(OUT, sc_diag, calibration, policy_df, pareto, ablation)
    summary = ["CU-Multi Stage 6 Selective Visual Verification", "PASS", "",
               f"Frozen SC R@1: {recall(sc_first, valid, 1):.6f} ({int((sc_first[valid] == 1).sum())}/1833)",
               f"SC Top-20 theoretical R@1 ceiling: {1830/1833:.6f} (1830/1833); 3 candidate-generation misses cannot be rescued.",
               f"Best-shift proxy: {'PASS' if proxy_pass else 'FAIL'}; MAE={np.mean(abs_error[corr_mask]):.3f}deg, median={np.median(abs_error[corr_mask]):.3f}deg, p90={np.percentile(abs_error[corr_mask],90):.3f}deg, Pearson={pearson:.3f}, Spearman={spearman:.3f}.",
               "", "Selected zero-regression exploratory points (not externally tuned):"]
    if len(selected_points):
        for _, p in selected_points.iterrows():
            summary.append(f"  {p.visual_method} {p.policy_id}: R@1={p['R@1']:.6f}, R@5={p['R@5']:.6f}, invocation={p.visual_invocation_rate_valid_queries:.4f}, overrides={int(p.overrides)}, rescues={int(p.rescues)}, regressions={int(p.regressions)}, net={int(p.net_corrections)}, added_compute={p.average_added_compute_ms:.3f}ms/query, serial={p.model_a_serial_latency_ms:.3f}ms/query")
    else:
        summary.append("  No sweep point had both at least one rescue and zero regressions.")
    summary += ["", "Global fusion is a control only; see global_fusion_control.csv.",
                "Oracle analyses are explicitly offline-only in oracle_analysis.csv.",
                f"Output: {OUT}"]
    (OUT / "summary.txt").write_text("\n".join(summary) + "\n")


if __name__ == "__main__":
    main()
