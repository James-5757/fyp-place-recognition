"""Freeze a small non-adjacent cohort using Stage24D RGB rankings only."""
from collections import defaultdict
from pathlib import Path

from stage26_manifest import read_csv, write_csv


def main():
    output = Path(__file__).resolve().parents[3] / 'outputs/cumulti_v1'
    dest = output / '26_rgbd_reconstruction'
    source = output / '24_rgbd_full_v1'
    groups = defaultdict(list)
    for row in read_csv(output/'24d_rgbd_pose_graph/candidates.csv'):
        if row['robot_a']==row['robot_b'] and int(row['epoch_b'])-int(row['epoch_a'])>1:
            groups[row['robot_a'],row['epoch_a'],row['epoch_b']].append(row)
    required = read_csv(dest/'stage26_missing_frames.csv')
    existing = {(row['robot'],row['index'],row['kind']) for row in required}
    plan = []
    for robot in ('robot1','robot3'):
        sync = {row['index']:row for row in read_csv(source/robot/'sync_pairs.csv')}
        ordered = sorted(((key,rows) for key,rows in groups.items() if key[0]==robot),
                         key=lambda item:(-max(float(r['score']) for r in item[1]), item[0]))
        count = 0
        for (_,ea,eb), rows in ordered:
            selection = []
            for row in sorted(rows,key=lambda r:int(r['rank'])):
                if any(row['index_a']==p['index_a'] or row['index_b']==p['index_b'] for p in selection):
                    continue
                selection.append(row)
            if len(selection)<6:
                continue
            case=f'{robot}:{ea}->{eb}'
            for offset,row in enumerate(selection[:6]):
                plan.append(dict(case=case,robot=robot,epoch_a=ea,epoch_b=eb,
                                 index_a=row['index_a'],index_b=row['index_b'],
                                 split='fit' if offset%2==0 else 'holdout',rank=row['rank'],score=row['score']))
                for side in ('a','b'):
                    index=row[f'index_{side}']
                    record=sync[index]
                    assert record['paired']=='True'
                    for kind, suffix in (('rgb','jpg'),('depth','png')):
                        key=(robot,index,kind)
                        if key in existing:
                            continue
                        existing.add(key)
                        required.append(dict(robot=robot,index=index,kind=kind,
                            expected_path=f'outputs/cumulti_v1/24_rgbd_full_v1/{robot}/{kind}/{int(index):06d}.{suffix}',
                            message_id=record[f'{kind}_message_id'],bag_ns=record[f'{kind}_bag_ns'],
                            header_ns=record[f'{kind}_header_ns'],recovery_sources='original camera ZIPs',
                            cases=case))
            count+=1
            if count==2:
                break
        assert count==2, robot
    write_csv(dest/'nonadjacent_candidates.csv',plan)
    write_csv(dest/'stage26_required_assets.csv',required)


if __name__=='__main__':
    main()
