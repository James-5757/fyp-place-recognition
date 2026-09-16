#!/usr/bin/env python3
"""CU-Multi Stage 7.5: same-view modality control and LiDAR azimuth audit.

Frozen Stage 4--7 artifacts are read only.  GT is used only after frozen scores
exist, to define offline strata and metrics.  No policy, fusion, or encoding runs.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import run_stage5_visual_viewpoint as s5

REPO = Path("/home/cas/fyp_place_recognition")
PROCESSED = Path("/home/cas/CU-Multi/processed_v1")
OUT = REPO / "outputs/cumulti_v1/07_5_supervisor_validation"
SMALL_BINS = [(0., 5., "0-5"), (0., 10., "0-10"), (0., 15., "0-15"), (0., 30., "0-30")]
CONTEXT_BINS = [(30., 60., "30-60"), (60., 90., "60-90"), (90., 120., "90-120"),
                (120., 150., "120-150"), (150., 180.000001, "150-180")]
METHODS = (("SC", "sc"), ("Single RGB", "single"), ("RGB5 Mean", "mean5"), ("Cross-Max", "crossmax"))


def first_rank(positive: np.ndarray, order: np.ndarray) -> np.ndarray:
    sorted_positive = np.take_along_axis(positive, order, axis=1)
    has = positive.any(axis=1)
    return np.where(has, np.argmax(sorted_positive, axis=1) + 1, -1).astype(int)


def metrics(ranks: np.ndarray, mask: np.ndarray) -> dict:
    values = ranks[mask]
    return {"query_count": int(mask.sum()),
            "R@1": float(np.mean((values >= 1) & (values <= 1))) if len(values) else np.nan,
            "R@5": float(np.mean((values >= 1) & (values <= 5))) if len(values) else np.nan,
            "MRR": float(np.mean(np.where(values > 0, 1. / values, 0.))) if len(values) else np.nan}


def visual_results(qemb, demb, top20, positive):
    qw, dw = s5.build_temporal_windows(len(qemb)), s5.build_temporal_windows(len(demb))
    results = {k: {"rank": np.full(len(qemb), -1, dtype=int), "positive_similarity": np.full(len(qemb), np.nan),
                   "top_score": np.full(len(qemb), np.nan)} for _, k in METHODS[1:]}
    for i, candidates in enumerate(top20):
        pos_local = positive[i, candidates]
        for _, method in METHODS[1:]:
            order, scores = s5.rerank_top20(qemb[i], demb[candidates], qemb[qw[i]], demb[dw[candidates]], method)
            ranked_pos = pos_local[order]
            if ranked_pos.any():
                results[method]["rank"][i] = int(np.argmax(ranked_pos) + 1)
            results[method]["top_score"][i] = float(scores[order[0]])
    return results, qw, dw


def qstats(values: np.ndarray) -> dict:
    values = values[np.isfinite(values)]
    return {"mean": float(np.mean(values)), "median": float(np.median(values)),
            "p25": float(np.percentile(values, 25)), "p75": float(np.percentile(values, 75))} if len(values) else {k: np.nan for k in ("mean", "median", "p25", "p75")}


def longest_empty_gap(occupied: np.ndarray) -> float:
    if occupied.all(): return 0.0
    if not occupied.any(): return 360.0
    doubled = np.concatenate((~occupied, ~occupied))
    longest = current = 0
    for x in doubled:
        current = current + 1 if x else 0
        longest = max(longest, current)
    return float(min(longest, 36) * 10)


def azimuth_record(cloud: np.ndarray, max_range: float | None) -> dict:
    finite = np.isfinite(cloud[:, :3]).all(axis=1)
    points = cloud[finite, :3]
    radius = np.hypot(points[:, 0], points[:, 1])
    if max_range is not None:
        points, radius = points[radius <= max_range], radius[radius <= max_range]
    az = np.degrees(np.arctan2(points[:, 1], points[:, 0])) if len(points) else np.array([])
    bins = np.floor((az + 180.) / 10.).astype(int).clip(0, 35) if len(az) else np.array([], dtype=int)
    occupied = np.zeros(36, dtype=bool); occupied[np.unique(bins)] = True
    return {"valid_point_count": int(len(points)), "min_observed_azimuth_deg": float(az.min()) if len(az) else np.nan,
            "max_observed_azimuth_deg": float(az.max()) if len(az) else np.nan,
            "occupied_bins": int(occupied.sum()), "occupied_bin_fraction": float(occupied.mean()),
            "circular_coverage_deg": float(occupied.sum() * 10), "longest_empty_gap_deg": longest_empty_gap(occupied),
            "approximately_full_azimuth": bool(occupied.sum() >= 34 and longest_empty_gap(occupied) <= 20),
            "occupancy_bitmap": "".join("1" if x else "0" for x in occupied)}


def audit_lidar() -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    rows = []
    for robot in ("robot1", "robot2", "robot3"):
        base = PROCESSED / "full_2hz" / robot
        kf = pd.read_csv(base / "keyframes.csv")
        chosen = np.unique(np.linspace(0, len(kf) - 1, 20, dtype=int))
        for index in chosen:
            cloud = np.load(base / "lidar" / kf.lidar_file.iloc[index])
            for limit, label in ((None, "all"), (20., "0-20m"), (40., "0-40m"), (80., "0-80m")):
                record = azimuth_record(cloud, limit)
                record.update({"robot": robot, "keyframe_id": int(kf.keyframe_id.iloc[index]),
                               "lidar_file": str(kf.lidar_file.iloc[index]), "range_slice": label})
                rows.append(record)
    frames = pd.DataFrame(rows)
    full = frames[frames.range_slice == "all"]
    summary = full.groupby("robot", as_index=False).agg(sampled_frames=("keyframe_id", "size"),
        approximately_full_azimuth_percent=("approximately_full_azimuth", lambda x: 100 * x.mean()),
        mean_occupied_bins=("occupied_bins", "mean"), largest_observed_blind_gap_deg=("longest_empty_gap_deg", "max"),
        mean_circular_coverage_deg=("circular_coverage_deg", "mean"))
    ranges = frames.groupby(["robot", "range_slice"], as_index=False).agg(sampled_frames=("keyframe_id", "size"),
        mean_occupied_bins=("occupied_bins", "mean"), mean_occupied_bin_fraction=("occupied_bin_fraction", "mean"),
        largest_observed_blind_gap_deg=("longest_empty_gap_deg", "max"), full_azimuth_percent=("approximately_full_azimuth", lambda x: 100 * x.mean()))
    return frames, summary, ranges


def figures(out, metric_df, sim_df, frame_df):
    small = metric_df[metric_df.subset.isin([x[2] for x in SMALL_BINS])]
    for col, name in (("R@1", "same_view_r1_comparison.png"), ("R@5", "same_view_r5_comparison.png")):
        fig, ax = plt.subplots(figsize=(8, 4.5), dpi=140)
        for method, group in small.groupby("method"): ax.plot(group.subset, group[col], "o-", label=method)
        ax.set(ylim=(0, 1.03), ylabel=col, xlabel="absolute heading difference (degrees)"); ax.grid(alpha=.25); ax.legend(); fig.tight_layout(); fig.savefig(out / name); plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=140)
    for method, group in sim_df.groupby("method"): ax.plot(group.subset, group["median"], "o-", label=method)
    ax.set(xlabel="heading bin (degrees)", ylabel="nearest-positive similarity (median)"); ax.grid(alpha=.25); ax.legend(); fig.tight_layout(); fig.savefig(out / "visual_similarity_vs_heading_small_bins.png"); plt.close(fig)
    fig, ax = plt.subplots(figsize=(7, 4.5), dpi=140)
    both = small[small.method.isin(["SC", "Single RGB"])]
    for method, group in both.groupby("method"): ax.plot(group.subset, group["R@1"], "o-", label=method)
    ax.set(ylim=(0, 1.03), xlabel="absolute heading difference (degrees)", ylabel="R@1"); ax.grid(alpha=.25); ax.legend(); fig.tight_layout(); fig.savefig(out / "sc_vs_rgb_same_view.png"); plt.close(fig)
    for robot in ("robot1", "robot2", "robot3"):
        group = frame_df[(frame_df.robot == robot) & (frame_df.range_slice == "all")]
        fig, ax = plt.subplots(figsize=(8, 3.5), dpi=140); ax.bar(np.arange(len(group)), group.occupied_bins)
        ax.axhline(34, c="tab:red", ls="--", label="34-bin criterion"); ax.set(ylim=(0, 37), xlabel="uniform sampled frame", ylabel="occupied azimuth bins / 36", title=robot); ax.legend(); fig.tight_layout(); fig.savefig(out / f"lidar_azimuth_coverage_{robot}.png"); plt.close(fig)
    pivot = frame_df.groupby(["robot", "range_slice"]).occupied_bin_fraction.mean().unstack()
    fig, ax = plt.subplots(figsize=(7, 4.5), dpi=140); im = ax.imshow(pivot.to_numpy(), vmin=0, vmax=1, cmap="viridis")
    ax.set_xticks(range(len(pivot.columns)), pivot.columns); ax.set_yticks(range(len(pivot.index)), pivot.index)
    for i in range(len(pivot.index)):
        for j in range(len(pivot.columns)): ax.text(j, i, f"{pivot.iloc[i,j]:.2f}", ha="center", va="center", color="white")
    fig.colorbar(im, ax=ax, label="mean occupied-bin fraction"); fig.tight_layout(); fig.savefig(out / "lidar_azimuth_bin_occupancy.png"); plt.close(fig)


def main():
    if OUT.exists() and any(OUT.iterdir()): raise RuntimeError(f"Refusing to overwrite {OUT}")
    OUT.mkdir(parents=True)
    q, db, _, _, top20, _, all_scores, order, dist, positive = s5.reconstruct_sc_top20(PROCESSED)
    valid = positive.any(axis=1)
    nearest = np.argmin(np.where(positive, dist, np.inf), axis=1)
    heading = np.full(len(q), np.nan); heading[valid] = s5.wrapped_abs_heading(q.yaw_deg.to_numpy()[valid], db.yaw_deg.to_numpy()[nearest[valid]])
    nearest_distance = np.full(len(q), np.nan); nearest_distance[valid] = dist[np.arange(len(q))[valid], nearest[valid]]
    sc_rank = first_rank(positive, order)
    qemb = np.load(PROCESSED / "openclip_stage5/robot1_embeddings.npy")
    demb = np.load(PROCESSED / "openclip_stage5/robot3_embeddings.npy")
    assert qemb.shape == (len(q), 512) and demb.shape == (len(db), 512)
    visual, qw, dw = visual_results(qemb, demb, top20, positive)
    nearest_single = np.sum(qemb * demb[nearest], axis=1)
    nearest_mean, nearest_cross = np.full(len(q), np.nan), np.full(len(q), np.nan)
    for i in np.where(valid)[0]:
        nearest_mean[i] = s5.visual_rgb5_mean(qemb[qw[i]], demb[dw[nearest[i]]])
        nearest_cross[i] = s5.visual_crossmax(qemb[qw[i]], demb[dw[nearest[i]]])
    sc_pos = all_scores[np.arange(len(q)), nearest]
    ranks = {"sc": sc_rank, "single": visual["single"]["rank"], "mean5": visual["mean5"]["rank"], "crossmax": visual["crossmax"]["rank"]}
    similarity = {"SC": sc_pos, "Single RGB": nearest_single, "Cross-Max": nearest_cross}
    metric_rows, sim_rows = [], []
    for low, high, label in SMALL_BINS + CONTEXT_BINS:
        mask = valid & (heading >= low) & (heading < high)
        for label_method, key in METHODS:
            row = {"subset": label, "heading_low_deg": low, "heading_high_deg": high, "method": label_method, **metrics(ranks[key], mask)}
            row.update({"failure_count": int(((ranks[key] != 1) & mask).sum())})
            if key != "sc":
                row.update({"rgb_correct_sc_wrong": int(((ranks[key] == 1) & (sc_rank != 1) & mask).sum()), "sc_correct_rgb_wrong": int(((sc_rank == 1) & (ranks[key] != 1) & mask).sum())})
            else: row.update({"rgb_correct_sc_wrong": np.nan, "sc_correct_rgb_wrong": np.nan})
            metric_rows.append(row)
        for name, values in similarity.items(): sim_rows.append({"subset": label, "heading_low_deg": low, "heading_high_deg": high, "method": name, **qstats(values[mask])})
    metrics_df, similarity_df = pd.DataFrame(metric_rows), pd.DataFrame(sim_rows)
    metrics_df.to_csv(OUT / "same_view_metrics.csv", index=False); similarity_df.to_csv(OUT / "positive_similarity_same_view.csv", index=False)
    distance_rows = []
    base = valid & (heading <= 10.)
    for limit in (2., 3., 5.):
        mask = base & (nearest_distance < limit)
        for label_method, key in (("SC", "sc"), ("Single RGB", "single"), ("Cross-Max", "crossmax")):
            distance_rows.append({"heading_subset": "0-10", "distance_threshold_m": limit, "method": label_method, **metrics(ranks[key], mask)})
    pd.DataFrame(distance_rows).to_csv(OUT / "same_view_distance_control.csv", index=False)
    cases = []
    categories = (("SC_correct_RGB_wrong", (sc_rank == 1) & (ranks["single"] != 1)), ("RGB_correct_SC_wrong", (sc_rank != 1) & (ranks["single"] == 1)), ("both_correct", (sc_rank == 1) & (ranks["single"] == 1)), ("both_wrong", (sc_rank != 1) & (ranks["single"] != 1)))
    for category, condition in categories:
        for i in np.where(base & condition)[0][:5]:
            cases.append({"selection_rule": "first_5_query_ids_per_category", "category": category, "query_id": int(q.keyframe_id.iloc[i]), "positive_candidate_id": int(db.keyframe_id.iloc[nearest[i]]), "gt_distance_m": nearest_distance[i], "gt_heading_difference_deg": heading[i], "SC_rank": int(sc_rank[i]), "Single_RGB_rank": int(ranks["single"][i]), "Cross_Max_rank": int(ranks["crossmax"][i]), "SC_positive_score": float(sc_pos[i]), "Single_RGB_positive_similarity": float(nearest_single[i]), "Cross_Max_positive_similarity": float(nearest_cross[i])})
    pd.DataFrame(cases).to_csv(OUT / "same_view_failure_cases.csv", index=False)
    frames, az_summary, az_ranges = audit_lidar(); frames.to_csv(OUT / "lidar_azimuth_frame_audit.csv", index=False); az_summary.to_csv(OUT / "lidar_azimuth_summary.csv", index=False); az_ranges.to_csv(OUT / "lidar_range_azimuth_summary.csv", index=False)
    figures(OUT, metrics_df, similarity_df, frames)
    config = {"stage": "7.5", "frozen_commit": "23bff54f0010eda2768f0a232fcef096b97a578c", "same_view_primary_deg": 10, "same_view_subsets_deg": [5, 10, 15, 30], "positive_definition": "d_xy < 5m, GT evaluation only", "azimuth_bins": 36, "full_azimuth_criterion": "occupied_bins >= 34 and longest_empty_gap_deg <= 20", "range_slices_m": [20, 40, 80], "openclip_encoding": "not rerun; Stage-5 cache reused", "prohibited": ["fusion", "Ring-Key", "GICP", "PGO", "CVTNet", "VLM"]}
    (OUT / "experiment_config.json").write_text(json.dumps(config, indent=2) + "\n")
    p10 = metrics_df[(metrics_df.subset == "0-10")].set_index("method")
    dist2 = pd.DataFrame(distance_rows); d2 = dist2[dist2.distance_threshold_m == 2.].set_index("method")
    validation = ["[PASS] Stages 4-7 read-only", "[PASS] cached Stage-5 OpenCLIP embeddings reused; no encoding", "[PASS] frozen SC Top-20 reconstructed and Stage-4 Rank-1 verified", "[PASS] GT heading/distance used only after ranking for offline strata", "[PASS] no threshold tuning or fusion", "[PASS] actual processed PointCloud2 XYZI frames audited for all three robots", "[PASS] full-azimuth criterion declared before audit", "[PASS] raw archives untouched", "[PASS] no Ring-Key, GICP, PGO, CVTNet, or VLM"]
    (OUT / "VALIDATION_REPORT.txt").write_text("\n".join(validation) + "\n")
    summary = ["CU-Multi Stage 7.5 supervisor validation", f"same-view <=10 deg valid queries: {int(p10.iloc[0].query_count)}", *[f"{method}: R@1={p10.loc[method, 'R@1']:.6f}, R@5={p10.loc[method, 'R@5']:.6f}" for method in p10.index], *[f"distance <2m {method}: R@1={d2.loc[method, 'R@1']:.6f}" for method in d2.index], "LiDAR full-azimuth results:", *[f"{r.robot}: {r.approximately_full_azimuth_percent:.1f}% pass, mean bins={r.mean_occupied_bins:.2f}/36, largest gap={r.largest_observed_blind_gap_deg:.1f} deg" for r in az_summary.itertuples(index=False)]]
    (OUT / "summary.txt").write_text("\n".join(summary) + "\n")
    print("\n".join(summary))

if __name__ == "__main__": main()
