import json
from pathlib import Path
import pandas as pd

ROOT = Path('/home/cas/fyp_place_recognition')
OUT = ROOT / 'outputs/canonical_v2/06_vlm'
THRESHOLD = 80

manifest = pd.read_csv(OUT / 'case_manifest.csv')
decisions = pd.read_csv(OUT / 'vlm_decisions.csv')
top = pd.read_csv(ROOT / 'outputs/canonical_v2/04_crossmax/canonical_top20_crossmax_scores.csv')
top['fusion_score'] = 0.6 * top['scan_context_score'] + 0.4 * top['temporal_cross_max']

# Offline-only pool status: distinguish no-overlap queries from canonical Top-20 misses.
pool_rows = []
for query_frame, group in top.groupby('query_frame'):
    query_frame = int(query_frame)
    if int(group['query_has_positive_in_B'].iloc[0]) == 0:
        status = 'NO_OVERLAP_QUERY'
        contains = False
    else:
        filtered = group.sort_values(['fusion_score', 'rank'], ascending=[False, True]).head(5)
        contains = bool(filtered['is_positive'].astype(int).any())
        status = 'POOL_CONTAINS_POSITIVE' if contains else 'FILTERED_POOL_MISS'
    pool_rows.append({'query_frame': query_frame, 'pool_status': status,
                      'filtered_top5_contains_positive': contains})
pool_df = pd.DataFrame(pool_rows)
pool_df.to_csv(OUT / 'pool_status.csv', index=False)
status_map = pool_df.set_index('query_frame')['pool_status'].to_dict()

keys = ['query_frame', 'case_type', 'anchor_frame', 'challenger_frame',
        'order', 'candidate_A_frame', 'candidate_B_frame']
merged = decisions.merge(manifest, on=keys, validate='one_to_one', suffixes=('', '_manifest'))

pair_rows = []
for pair_key, group in merged.groupby(
    ['query_frame', 'case_type', 'anchor_frame', 'challenger_frame'], sort=True
):
    if len(group) != 2 or set(group['order']) != {'AB', 'BA'}:
        raise ValueError(f'Incomplete AB/BA pair: {pair_key}')
    ab = group[group['order'] == 'AB'].iloc[0]
    ba = group[group['order'] == 'BA'].iloc[0]
    ab_frame = int(ab['chosen_frame']) if pd.notna(ab['chosen_frame']) else None
    ba_frame = int(ba['chosen_frame']) if pd.notna(ba['chosen_frame']) else None
    raw_consistent = bool(ab['api_success']) and bool(ba['api_success']) and \
        ab['choice'] in {'A', 'B'} and ba['choice'] in {'A', 'B'} and \
        ab_frame is not None and ab_frame == ba_frame
    raw_winner = ab_frame if raw_consistent else None
    min_confidence = min(int(ab['confidence']), int(ba['confidence']))
    actionable = bool(raw_consistent and min_confidence >= THRESHOLD)
    source = group.iloc[0]
    winner_positive = None
    if raw_winner == int(source['candidate_A_frame']):
        winner_positive = int(source['candidate_A_is_positive'])
    elif raw_winner == int(source['candidate_B_frame']):
        winner_positive = int(source['candidate_B_is_positive'])
    pair_rows.append({
        'query_frame': int(pair_key[0]), 'case_type': pair_key[1],
        'anchor_frame': int(pair_key[2]), 'challenger_frame': int(pair_key[3]),
        'AB_choice': ab['choice'], 'BA_choice': ba['choice'],
        'AB_chosen_frame': ab_frame, 'BA_chosen_frame': ba_frame,
        'RAW_CONSISTENT_WINNER': raw_winner,
        'HIGH_CONFIDENCE_ACTIONABLE_WINNER': raw_winner if actionable else None,
        'raw_consistent_winner': raw_consistent,
        'high_confidence_actionable_winner': actionable,
        'min_confidence': min_confidence, 'winner_is_positive': winner_positive,
        'anchor_is_positive': int(source['anchor_is_positive']),
        'challenger_is_positive': int(source['challenger_is_positive']),
        'api_success_pair': bool(ab['api_success']) and bool(ba['api_success']),
    })
pair_df = pd.DataFrame(pair_rows)
pair_df.to_csv(OUT / 'pair_evaluation.csv', index=False)

query_rows = []
for query_frame, group in pair_df.groupby('query_frame', sort=True):
    query_frame = int(query_frame)
    anchor_positive = bool(group['anchor_is_positive'].iloc[0])
    actionable_challengers = group[
        group['high_confidence_actionable_winner'] &
        (group['HIGH_CONFIDENCE_ACTIONABLE_WINNER'] == group['challenger_frame'])
    ]
    actionable_frames = sorted({int(v) for v in actionable_challengers[
        'HIGH_CONFIDENCE_ACTIONABLE_WINNER'].dropna()})
    deployment_override = len(actionable_frames) == 1
    selected_frame = actionable_frames[0] if deployment_override else None
    selected_rows = actionable_challengers[
        actionable_challengers['HIGH_CONFIDENCE_ACTIONABLE_WINNER'] == selected_frame
    ] if deployment_override else actionable_challengers.iloc[0:0]
    selected_positive = (bool(int(selected_rows['winner_is_positive'].iloc[0]))
                         if deployment_override else None)
    raw_positive = group[
        group['raw_consistent_winner'] &
        (group['RAW_CONSISTENT_WINNER'] == group['challenger_frame']) &
        (group['winner_is_positive'] == 1)
    ]
    correction = bool(deployment_override and not anchor_positive and selected_positive)
    regression = bool(deployment_override and anchor_positive and not selected_positive)
    status = status_map[query_frame]
    action = ('FILTERED_POOL_MISS' if status == 'FILTERED_POOL_MISS' else
              'NO_OVERLAP_QUERY' if status == 'NO_OVERLAP_QUERY' else
              'OVERRIDE_SC' if deployment_override else 'KEEP_SC')
    query_rows.append({
        'query_frame': query_frame, 'pool_status': status,
        'anchor_is_positive': anchor_positive,
        'raw_consistent_winner_frames': json.dumps([
            int(v) for v in group['RAW_CONSISTENT_WINNER'].dropna().unique()
        ]),
        'raw_positive_challengers': json.dumps([
            int(v) for v in raw_positive['RAW_CONSISTENT_WINNER'].dropna().unique()
        ]),
        'actionable_winner_frames': json.dumps(actionable_frames),
        'actionable_positive_challengers': json.dumps([
            int(v) for v in actionable_challengers[
                actionable_challengers['winner_is_positive'] == 1
            ]['HIGH_CONFIDENCE_ACTIONABLE_WINNER'].dropna().unique()
        ]),
        'deployment_actionable_challenger': selected_frame,
        'actionable_winner_is_positive': selected_positive,
        'threshold_abstained_correct': bool(len(raw_positive) and not deployment_override),
        'action': action, 'system_correction': correction,
        'system_regression': regression,
    })
query_df = pd.DataFrame(query_rows)
query_df.to_csv(OUT / 'query_evaluation.csv', index=False)

case_rows = []
for case_type, group in pair_df.groupby('case_type', sort=True):
    case_rows.append({
        'case_type': case_type, 'comparisons': len(group),
        'api_success_rate': float(group['api_success_pair'].mean()),
        'ab_ba_consistency_rate': float(group['raw_consistent_winner'].mean()),
        'abstention_rate': float(1 - group['high_confidence_actionable_winner'].mean()),
        'raw_positive_winner_rate': float((group['winner_is_positive'] == 1).mean()),
        'actionable_positive_winner_rate': float(
            (group['high_confidence_actionable_winner'] &
             (group['winner_is_positive'] == 1)).mean()
        ),
    })
pd.DataFrame(case_rows).to_csv(OUT / 'case_metrics.csv', index=False)

corrections = int(query_df['system_correction'].sum())
regressions = int(query_df['system_regression'].sum())
pool_misses = int((query_df['pool_status'] == 'FILTERED_POOL_MISS').sum())
no_overlap = int((query_df['pool_status'] == 'NO_OVERLAP_QUERY').sum())
abstained_correct = int(query_df['threshold_abstained_correct'].sum())
system_top1 = 149 + corrections - regressions
classification = (
    'SELECTIVE_VLM_ADDS_VALUE' if corrections >= 1 and regressions == 0 else
    'VLM_VERIFICATION_INTRODUCES_TOO_MANY_REGRESSIONS'
    if regressions > corrections else
    'VLM_USEFUL_BUT_CONFIDENCE_OR_STABILITY_LIMITED'
    if abstained_correct else 'VLM_SEMANTIC_VERIFICATION_NOT_EFFECTIVE'
)
lines = [
    '# Canonical Selective VLM Evaluation', '',
    f'Canonical Top20 pool misses: {pool_misses}',
    f'No-overlap queries: {no_overlap}',
    f'AB/BA consistency: {pair_df.raw_consistent_winner.mean():.6f}',
    f'Actionable rate: {pair_df.high_confidence_actionable_winner.mean():.6f}',
    f'Corrections: {corrections}', f'Regressions: {regressions}',
    f'Net gain: {corrections - regressions}',
    f'System R@1: {system_top1}/158 = {system_top1 / 158:.6f}',
    f'Threshold-abstained raw positive decisions: {abstained_correct}',
    f'Classification: {classification}',
]
(OUT / 'summary.txt').write_text('\n'.join(lines) + '\n')
print('\n'.join(lines))
print('pair_rows', len(pair_df), 'query_rows', len(query_df))
print('pool_status_counts', query_df['pool_status'].value_counts().to_dict())
