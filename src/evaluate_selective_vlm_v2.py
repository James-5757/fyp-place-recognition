import argparse
import json
from pathlib import Path

import pandas as pd

THRESHOLD = 80


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=Path("outputs/selective_vlm_v2/manifest.csv"))
    parser.add_argument("--decisions", type=Path, default=Path("outputs/selective_vlm_v2/vlm_decisions.csv"))
    parser.add_argument("--pool-status", type=Path, default=Path("outputs/selective_vlm_v2/pool_status.csv"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/selective_vlm_v2"))
    args = parser.parse_args()

    manifest = pd.read_csv(args.manifest)
    decisions = pd.read_csv(args.decisions)
    pool_status = pd.read_csv(args.pool_status)
    keys = ["query_frame", "case_type", "anchor_frame", "challenger_frame", "order", "candidate_A_frame", "candidate_B_frame"]
    merged = decisions.merge(manifest, on=keys, validate="one_to_one", suffixes=("", "_manifest"))
    pairs = []
    for key, group in merged.groupby(["query_frame", "case_type", "anchor_frame", "challenger_frame"], sort=True):
        if len(group) != 2:
            continue
        ab = group[group.order == "AB"].iloc[0]
        ba = group[group.order == "BA"].iloc[0]
        ab_frame = int(ab.chosen_frame) if pd.notna(ab.chosen_frame) else None
        ba_frame = int(ba.chosen_frame) if pd.notna(ba.chosen_frame) else None
        raw_consistent = bool(ab.api_success) and bool(ba.api_success) and ab.choice in {"A", "B"} and ba.choice in {"A", "B"} and ab_frame == ba_frame
        raw_winner = ab_frame if raw_consistent else None
        min_confidence = min(int(ab.confidence), int(ba.confidence))
        actionable = raw_consistent and min_confidence >= THRESHOLD
        row = group.iloc[0]
        winner_positive = None
        if raw_winner == int(row.candidate_A_frame):
            winner_positive = int(row.candidate_A_is_positive)
        elif raw_winner == int(row.candidate_B_frame):
            winner_positive = int(row.candidate_B_is_positive)
        pairs.append({
            "query_frame": int(key[0]), "case_type": key[1], "anchor_frame": int(key[2]), "challenger_frame": int(key[3]),
            "AB_choice": ab.choice, "BA_choice": ba.choice, "AB_chosen_frame": ab_frame, "BA_chosen_frame": ba_frame,
            "RAW_CONSISTENT_WINNER": raw_winner, "HIGH_CONFIDENCE_ACTIONABLE_WINNER": raw_winner if actionable else None,
            "raw_consistent_winner": raw_consistent, "high_confidence_actionable_winner": actionable,
            "min_confidence": min_confidence, "winner_is_positive": winner_positive,
            "api_success_pair": bool(ab.api_success) and bool(ba.api_success),
            "anchor_is_positive": int(row.anchor_is_positive), "challenger_is_positive": int(row.challenger_is_positive),
        })
    pair_df = pd.DataFrame(pairs)
    pair_df.to_csv(args.output_dir / "pair_evaluation.csv", index=False)

    statuses = pool_status.set_index("query_frame").pool_status.to_dict()
    query_rows = []
    for query, group in pair_df.groupby("query_frame", sort=True):
        raw_positive = group[(group.raw_consistent_winner) & (group.RAW_CONSISTENT_WINNER == group.challenger_frame) & (group.winner_is_positive == 1)]
        anchor_positive = bool(group.iloc[0].anchor_is_positive)
        actionable_challenger = group[
            (group.high_confidence_actionable_winner)
            & (group.HIGH_CONFIDENCE_ACTIONABLE_WINNER == group.challenger_frame)
        ]
        actionable_positive = actionable_challenger[
            actionable_challenger.winner_is_positive == 1
        ]
        actionable_negative = actionable_challenger[
            actionable_challenger.winner_is_positive == 0
        ]
        actionable_frames = sorted(
            int(frame)
            for frame in actionable_challenger.HIGH_CONFIDENCE_ACTIONABLE_WINNER
            .dropna()
            .astype(int)
            .unique()
        )
        # Deployment uses no GT: only one unambiguous high-confidence challenger can override.
        deployment_override = len(actionable_frames) == 1
        selected_challenger = actionable_frames[0] if deployment_override else pd.NA
        selected_row = actionable_challenger[
            actionable_challenger.HIGH_CONFIDENCE_ACTIONABLE_WINNER == selected_challenger
        ] if deployment_override else actionable_challenger.iloc[0:0]
        selected_is_positive = (
            bool(int(selected_row.iloc[0].winner_is_positive))
            if deployment_override
            else pd.NA
        )
        system_correction = bool(
            deployment_override and not anchor_positive and selected_is_positive
        )
        system_regression = bool(
            deployment_override and anchor_positive and not selected_is_positive
        )
        status = statuses[int(query)]
        if status == "FILTERED_POOL_MISS":
            action = "FILTERED_POOL_MISS"
        elif deployment_override:
            action = "OVERRIDE_SC"
        else:
            action = "KEEP_SC"
        query_rows.append({
            "query_frame": int(query), "pool_status": status,
            "raw_positive_challengers": json.dumps([int(v) for v in raw_positive.RAW_CONSISTENT_WINNER.dropna().unique()]),
            "actionable_positive_challengers": json.dumps([int(v) for v in actionable_positive.HIGH_CONFIDENCE_ACTIONABLE_WINNER.dropna().unique()]),
            "deployment_actionable_challenger": selected_challenger,
            "deployment_override": deployment_override,
            "actionable_winner_is_positive": selected_is_positive,
            "threshold_abstained_correct": bool(len(raw_positive) and not len(actionable_positive)),
            "anchor_is_positive": anchor_positive,
            "action": action, "system_correction": system_correction, "system_regression": system_regression,
        })
    query_df = pd.DataFrame(query_rows)
    query_df.to_csv(args.output_dir / "query_evaluation.csv", index=False)

    metric_rows = []
    for case_type, group in pair_df.groupby("case_type"):
        metric_rows.append({
            "case_type": case_type, "comparisons": len(group),
            "api_success_rate": group.api_success_pair.mean(),
            "ab_ba_consistency_rate": group.raw_consistent_winner.mean(),
            "abstention_rate": 1 - group.high_confidence_actionable_winner.mean(),
            "raw_positive_winner_rate": ((group.raw_consistent_winner) & (group.winner_is_positive == 1)).mean(),
            "actionable_positive_winner_rate": ((group.high_confidence_actionable_winner) & (group.winner_is_positive == 1)).mean(),
        })
    pd.DataFrame(metric_rows).to_csv(args.output_dir / "case_metrics.csv", index=False)

    corrections = int(query_df.system_correction.sum())
    regressions = int(query_df.system_regression.sum())
    raw_rejected = int(query_df.threshold_abstained_correct.sum())
    pool_misses = int((query_df.pool_status == "FILTERED_POOL_MISS").sum())
    system_correct = 151 + corrections - regressions
    valid_pairs = pair_df[pair_df.api_success_pair]
    classification = (
        "SELECTIVE_VLM_ADDS_VALUE" if corrections >= 1 and regressions == 0
        else "VLM_VERIFICATION_INTRODUCES_TOO_MANY_REGRESSIONS" if regressions > corrections
        else "VLM_USEFUL_BUT_CONFIDENCE_OR_STABILITY_LIMITED" if raw_rejected >= 1
        else "VLM_SEMANTIC_VERIFICATION_NOT_EFFECTIVE"
    )
    lines = [
        "SELECTIVE VLM VERIFICATION V2", "Candidate stage:",
        "SC Top20 recall: 158/158 = 100%", "SC+CrossMax Top5 retention: 157/158 = 99.37%",
        f"Filtered-pool misses: {pool_misses}", "VLM evaluation:",
        f"disagreement queries = {manifest[manifest.case_type == 'DISAGREEMENT'].query_frame.nunique()}",
        f"control queries = {manifest[manifest.case_type.str.contains('CONTROL')].query_frame.nunique()}",
        f"AB/BA consistency = {pair_df.raw_consistent_winner.mean():.6f}",
        f"abstention rate = {1 - pair_df.high_confidence_actionable_winner.mean():.6f}",
        f"Raw correct VLM decisions = {int(((pair_df.raw_consistent_winner) & (pair_df.winner_is_positive == 1)).sum())}",
        f"High-confidence actionable corrections = {corrections}", f"High-confidence regressions = {regressions}",
        f"Net system gain = {corrections - regressions}", f"System Top1 after selective VLM: {system_correct} / 158",
        f"R@1 = {system_correct / 158:.6f}", f"candidate filtering failure count = {pool_misses}",
        f"VLM verification failure count = {int((query_df.pool_status == 'POOL_CONTAINS_POSITIVE').sum()) - corrections}",
        f"potential corrections rejected by confidence threshold = {raw_rejected}",
        f"classification: {classification}",
    ]
    for query in [1545, 1550, 1560, 115, 955, 1565, 380]:
        row = query_df[query_df.query_frame == query]
        if len(row):
            row = row.iloc[0]
            lines.append(f"Query {query}: pool={row.pool_status}, raw={row.raw_positive_challengers}, actionable={row.actionable_positive_challengers}, action={row.action}")
    (args.output_dir / "summary.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
