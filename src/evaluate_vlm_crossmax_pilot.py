import argparse
import json
from pathlib import Path

import pandas as pd


THRESHOLD = 80


def args():
    parser = argparse.ArgumentParser(description="Evaluate Cross-Max VLM pilot")
    parser.add_argument("--manifest", type=Path, default=Path("outputs/vlm_crossmax_pilot/manifest.csv"))
    parser.add_argument("--decisions", type=Path, default=Path("outputs/vlm_crossmax_pilot/vlm_decisions.csv"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/vlm_crossmax_pilot"))
    return parser.parse_args()


def main():
    parsed = args()
    manifest = pd.read_csv(parsed.manifest)
    decisions = pd.read_csv(parsed.decisions)
    merged = decisions.merge(
        manifest,
        on=["query_frame", "sc_rank1_frame", "challenger_frame", "order", "candidate_A_frame", "candidate_B_frame"],
        validate="one_to_one",
        suffixes=("", "_manifest"),
    )
    rows = []
    for key, group in merged.groupby(["query_frame", "sc_rank1_frame", "challenger_frame"], sort=True):
        if len(group) != 2:
            raise ValueError(f"Expected AB/BA for {key}")
        ab = group[group.order == "AB"].iloc[0]
        ba = group[group.order == "BA"].iloc[0]
        ab_frame = int(ab.chosen_frame) if pd.notna(ab.chosen_frame) else None
        ba_frame = int(ba.chosen_frame) if pd.notna(ba.chosen_frame) else None
        raw_consistent = ab.choice in {"A", "B"} and ba.choice in {"A", "B"} and ab_frame is not None and ab_frame == ba_frame
        raw_winner = ab_frame if raw_consistent else None
        min_confidence = min(int(ab.confidence), int(ba.confidence))
        actionable = raw_consistent and min_confidence >= THRESHOLD
        winner_positive = int(group.iloc[0].candidate_A_is_positive) if raw_winner == int(group.iloc[0].candidate_A_frame) else (int(group.iloc[0].candidate_B_is_positive) if raw_winner == int(group.iloc[0].candidate_B_frame) else None)
        rows.append({
            "query_frame": int(key[0]),
            "sc_rank1_frame": int(key[1]),
            "challenger_frame": int(key[2]),
            "challenger_original_rank": int(group.iloc[0].challenger_original_rank),
            "crossmax_score": float(group.iloc[0].crossmax_score),
            "best_query_temporal_frame": int(group.iloc[0].best_query_temporal_frame),
            "best_candidate_temporal_frame": int(group.iloc[0].best_candidate_temporal_frame),
            "AB_choice": ab.choice,
            "BA_choice": ba.choice,
            "AB_chosen_frame": ab_frame,
            "BA_chosen_frame": ba_frame,
            "RAW_CONSISTENT_WINNER": raw_winner,
            "HIGH_CONFIDENCE_ACTIONABLE_WINNER": raw_winner if actionable else None,
            "raw_consistent_winner": raw_consistent,
            "high_confidence_actionable_winner": actionable,
            "min_confidence": min_confidence,
            "winner_is_positive": winner_positive,
        })
    pairs = pd.DataFrame(rows)
    pairs.to_csv(parsed.output_dir / "pair_evaluation.csv", index=False)
    qrows = []
    for query, group in pairs.groupby("query_frame", sort=True):
        raw_positive = group[(group.raw_consistent_winner == True) & (group.RAW_CONSISTENT_WINNER == group.challenger_frame) & (group.winner_is_positive == 1)]
        actionable_positive = group[(group.high_confidence_actionable_winner == True) & (group.HIGH_CONFIDENCE_ACTIONABLE_WINNER == group.challenger_frame) & (group.winner_is_positive == 1)]
        if len(actionable_positive) == 1:
            action = "OVERRIDE_SC"
        elif len(raw_positive) == 1:
            action = "RAW_CORRECT_THRESHOLD_ABSTAINED"
        else:
            action = "KEEP_SC"
        qrows.append({
            "query_frame": int(query),
            "sc_rank1_frame": int(group.iloc[0].sc_rank1_frame),
            "crossmax_challenger_frame": int(group.iloc[0].challenger_frame),
            "raw_consistent_winner": json.dumps([int(x) for x in group.RAW_CONSISTENT_WINNER.dropna().astype(int)]),
            "high_confidence_actionable_winner": json.dumps([int(x) for x in group.HIGH_CONFIDENCE_ACTIONABLE_WINNER.dropna().astype(int)]),
            "raw_positive_challenger": bool(len(raw_positive)),
            "high_confidence_positive_challenger": bool(len(actionable_positive)),
            "min_confidence": int(group.min_confidence.min()),
            "action": action,
        })
    query = pd.DataFrame(qrows)
    query.to_csv(parsed.output_dir / "query_evaluation.csv", index=False)
    lines = ["CROSS-MAX-GUIDED VLM SEMANTIC VERIFICATION PILOT"]
    for row in query.itertuples(index=False):
        lines += [
            f"Query {row.query_frame}:",
            f"SC Rank1 = {row.sc_rank1_frame}",
            f"Cross-Max challenger = {row.crossmax_challenger_frame}",
            f"RAW_CONSISTENT_WINNER = {row.raw_consistent_winner}",
            f"HIGH_CONFIDENCE_ACTIONABLE_WINNER = {row.high_confidence_actionable_winner}",
            f"min confidence = {row.min_confidence}",
            f"action = {row.action}",
        ]
    raw_count = int(query.raw_positive_challenger.sum())
    actionable_count = int(query.high_confidence_positive_challenger.sum())
    inconsistent = int((~pairs.raw_consistent_winner).sum())
    uncertain = int(decisions.choice.isin(["UNCERTAIN", "NEITHER"]).sum())
    api_errors = int((~decisions.request_success.astype(bool)).sum())
    model_uncertain_neither = int(
        decisions[decisions.request_success.astype(bool)].choice.isin(
            ["UNCERTAIN", "NEITHER"]
        ).sum()
    )
    pilot_valid = api_errors == 0 and len(decisions) == len(manifest)
    recommendation = (
        "PROCEED_TO_VLM_CONTROL_SET"
        if pilot_valid and actionable_count >= 2
        else "VLM_SEMANTIC_VERIFICATION_NOT_RELIABLE_YET"
    )
    lines += [
        "Overall:",
        f"Cross-Max positive challengers raw-consistent = {raw_count}/3",
        f"Cross-Max high-confidence actionable positives = {actionable_count}/3",
        f"A/B order inconsistencies = {inconsistent}",
        f"API_ERROR_COUNT = {api_errors}",
        f"model_uncertain_neither_decisions = {model_uncertain_neither}",
        f"pilot_valid = {pilot_valid}",
        f"recommendation: {recommendation}",
    ]
    (parsed.output_dir / "summary.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
