import argparse
import json
from pathlib import Path

import pandas as pd


CONFIDENCE_THRESHOLD = 80
QUERIES = [1545, 1550, 1560]


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate VLM verification pilot")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("outputs/vlm_verification_pilot/manifest.csv"),
    )
    parser.add_argument(
        "--decisions",
        type=Path,
        default=Path("outputs/vlm_verification_pilot/vlm_decisions.csv"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/vlm_verification_pilot"),
    )
    return parser.parse_args()


def choice_for_order(group, order):
    rows = group[group.order == order]
    if len(rows) != 1:
        raise ValueError(f"Expected one {order} row; found {len(rows)}")
    return rows.iloc[0]


def positive_for_frame(manifest_group, frame):
    for row in manifest_group.itertuples(index=False):
        if int(row.candidate_A_frame) == frame:
            return int(row.candidate_A_is_positive)
        if int(row.candidate_B_frame) == frame:
            return int(row.candidate_B_is_positive)
    raise ValueError(f"Frame {frame} absent from manifest pair")


def main():
    args = parse_args()
    for path in [args.manifest, args.decisions]:
        if not path.is_file():
            raise FileNotFoundError(path)
    manifest = pd.read_csv(args.manifest)
    decisions = pd.read_csv(args.decisions)
    keys = ["query_frame", "sc_rank1_frame", "challenger_frame", "order"]
    if decisions.duplicated(keys).any():
        raise ValueError("Duplicate decision rows")
    merged = decisions.merge(
        manifest,
        on=[
            "query_frame",
            "sc_rank1_frame",
            "challenger_frame",
            "order",
            "candidate_A_frame",
            "candidate_B_frame",
        ],
        how="left",
        validate="one_to_one",
        suffixes=("", "_manifest"),
    )
    if len(merged) != 24:
        raise ValueError(f"Expected 24 complete decisions, found {len(merged)}")

    pair_rows = []
    pair_keys = ["query_frame", "sc_rank1_frame", "challenger_frame"]
    for key, group in merged.groupby(pair_keys, sort=True):
        if len(group) != 2:
            raise ValueError(f"Expected AB and BA rows for pair {key}")
        ab = choice_for_order(group, "AB")
        ba = choice_for_order(group, "BA")
        ab_chosen = int(ab.chosen_frame) if pd.notna(ab.chosen_frame) else None
        ba_chosen = int(ba.chosen_frame) if pd.notna(ba.chosen_frame) else None
        raw_consistent = (
            ab.choice in {"A", "B"}
            and ba.choice in {"A", "B"}
            and ab_chosen is not None
            and ab_chosen == ba_chosen
        )
        raw_winner = ab_chosen if raw_consistent else None
        min_confidence = min(int(ab.confidence), int(ba.confidence))
        actionable = raw_consistent and min_confidence >= CONFIDENCE_THRESHOLD
        decision = "WINNER" if actionable else "ABSTAIN"
        rank1_positive = int(group.iloc[0].candidate_A_is_positive)
        if int(group.iloc[0].candidate_A_frame) != int(key[1]):
            rank1_positive = int(group.iloc[0].candidate_B_is_positive)
        winner_positive = (
            positive_for_frame(group, raw_winner)
            if raw_winner is not None
            else pd.NA
        )
        pair_rows.append(
            {
                "query_frame": int(key[0]),
                "sc_rank1_frame": int(key[1]),
                "challenger_frame": int(key[2]),
                "challenger_rank": int(group.iloc[0].challenger_original_rank),
                "AB_choice": ab.choice,
                "BA_choice": ba.choice,
                "AB_chosen_frame": ab_chosen,
                "BA_chosen_frame": ba_chosen,
                "RAW_CONSISTENT_WINNER": raw_winner,
                "HIGH_CONFIDENCE_ACTIONABLE_WINNER": raw_winner if actionable else None,
                "raw_consistent_winner": raw_consistent,
                "high_confidence_actionable_winner": actionable,
                "consistent": raw_consistent,
                "winner_frame": raw_winner,
                "min_confidence": min_confidence,
                "decision": decision,
                "winner_is_positive": winner_positive,
                "sc_rank1_is_positive": rank1_positive,
            }
        )
    pairs = pd.DataFrame(pair_rows)
    pairs.to_csv(args.output_dir / "pair_evaluation.csv", index=False)

    query_rows = []
    for query_frame in QUERIES:
        group = pairs[pairs.query_frame == query_frame]
        rank1_frame = int(group.iloc[0].sc_rank1_frame)
        rank1_positive = int(group.iloc[0].sc_rank1_is_positive)
        raw_challenger_wins = group[
            group.RAW_CONSISTENT_WINNER == group.challenger_frame
        ]
        raw_positive_wins = raw_challenger_wins[
            raw_challenger_wins.winner_is_positive.astype("boolean") == True
        ]
        actionable_challenger_wins = group[
            (group.HIGH_CONFIDENCE_ACTIONABLE_WINNER == group.challenger_frame)
            & (group.decision == "WINNER")
        ]
        actionable_positive_wins = actionable_challenger_wins[
            actionable_challenger_wins.winner_is_positive.astype("boolean") == True
        ]
        raw_winner_frames = [
            int(value)
            for value in sorted(
                raw_challenger_wins.RAW_CONSISTENT_WINNER.dropna()
                .astype(int)
                .unique()
            )
        ]
        raw_positive_frames = [
            int(value)
            for value in sorted(
                raw_positive_wins.RAW_CONSISTENT_WINNER.dropna()
                .astype(int)
                .unique()
            )
        ]
        actionable_frames = [
            int(value)
            for value in sorted(
                actionable_challenger_wins.HIGH_CONFIDENCE_ACTIONABLE_WINNER.dropna()
                .astype(int)
                .unique()
            )
        ]
        actionable_positive_frames = [
            int(value)
            for value in sorted(
                actionable_positive_wins.HIGH_CONFIDENCE_ACTIONABLE_WINNER.dropna()
                .astype(int)
                .unique()
            )
        ]
        if len(actionable_positive_frames) == 1 and len(actionable_frames) == 1:
            action = "OVERRIDE_SC"
            selected_frame = actionable_positive_frames[0]
        elif len(raw_positive_frames) == 1 and len(raw_winner_frames) == 1:
            action = "RAW_CORRECT_THRESHOLD_ABSTAINED"
            selected_frame = pd.NA
        elif len(actionable_challenger_wins) == 0:
            action = "KEEP_SC"
            selected_frame = rank1_frame
        else:
            action = "ABSTAIN"
            selected_frame = pd.NA
        query_rows.append(
            {
                "query_frame": query_frame,
                "sc_rank1_frame": rank1_frame,
                "sc_rank1_is_positive": rank1_positive,
                "raw_consistent_winner_frames": json.dumps(raw_winner_frames),
                "raw_positive_challenger_frames": json.dumps(raw_positive_frames),
                "actionable_winner_frames": json.dumps(actionable_frames),
                "actionable_positive_challenger_frames": json.dumps(actionable_positive_frames),
                "positive_challengers_discovered": bool(raw_positive_frames),
                "high_confidence_positive_challenger": bool(actionable_positive_frames),
                "action": action,
                "selected_frame": selected_frame,
            }
        )
    query_summary = pd.DataFrame(query_rows)
    query_summary.to_csv(args.output_dir / "query_evaluation.csv", index=False)

    consistent_positive_queries = int(
        query_summary.positive_challengers_discovered.sum()
    )
    potential_corrections = int((query_summary.action == "OVERRIDE_SC").sum())
    inconsistencies = int((~pairs.raw_consistent_winner.astype(bool)).sum())
    abstentions = int(
        decisions.choice.isin(["UNCERTAIN", "NEITHER"]).sum()
    )
    stable_pair_rate = float(pairs.consistent.mean())
    recommendation = (
        "PROCEED_TO_VLM_CONTROL_SET"
        if potential_corrections >= 2 and stable_pair_rate >= 0.75
        else "VLM_SEMANTIC_VERIFICATION_NOT_RELIABLE_YET"
    )

    lines = ["VLM SEMANTIC VERIFICATION PILOT"]
    for row in query_summary.itertuples(index=False):
        group = pairs[pairs.query_frame == row.query_frame]
        positive = group[
            (group.raw_consistent_winner.astype(bool))
            & (group.winner_is_positive.astype("boolean") == True)
        ]
        min_confidence = (
            int(positive.min_confidence.min()) if len(positive) else 0
        )
        raw_positive = bool(len(positive))
        lines.extend(
            [
                f"Query {row.query_frame}:",
                f"SC Rank1 = {row.sc_rank1_frame}",
                f"SC Rank1 positive = {bool(row.sc_rank1_is_positive)}",
                f"RAW_CONSISTENT_WINNER = {row.raw_positive_challenger_frames}",
                f"HIGH_CONFIDENCE_ACTIONABLE_WINNER = {row.actionable_positive_challenger_frames}",
                f"raw correct = {'yes' if raw_positive else 'no'}",
                f"VLM selected = {row.selected_frame if pd.notna(row.selected_frame) else 'ABSTAIN'}",
                f"AB/BA raw consistent = {bool(group.raw_consistent_winner.all())}",
                f"min confidence = {min_confidence}",
                f"high-confidence actionable = {'yes' if row.high_confidence_positive_challenger else 'no'}",
                f"potential correction = {'yes' if row.action == 'OVERRIDE_SC' else 'no'}",
                f"action = {row.action}",
            ]
        )
    lines.extend(
        [
            "Overall:",
            "recoverable SC failures tested = 3",
            f"positive challengers identified consistently = {consistent_positive_queries}/3",
            f"high-confidence potential corrections = {potential_corrections}/3",
            f"A/B order inconsistencies = {inconsistencies}",
            f"uncertain/neither decisions = {abstentions}",
            f"recommendation: {recommendation}",
        ]
    )
    summary = "\n".join(lines)
    (args.output_dir / "summary.txt").write_text(summary + "\n")
    print(summary)


if __name__ == "__main__":
    main()
