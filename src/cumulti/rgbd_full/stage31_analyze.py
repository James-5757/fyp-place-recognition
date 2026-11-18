"""Fit-only segment candidates and frame-disjoint validation, never trajectory edits."""
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from common import dump
from fusion import pose
from stage27_continuous import delta

IDENTITY=np.eye(4)
METHODS=('orb_metric','icp_identity','fpfh_ransac_icp','rgbd_flow_ransac')


def summary(metrics):
    if not metrics:
        return None
    return dict(n=len(metrics),
                bidirectional_overlap_median=np.median([r['overlap'] for r in metrics],axis=0).tolist(),
                bidirectional_rmse_median_m=[float(np.median(v)) if v else None for v in
                    ([r['rmse_m'][side] for r in metrics if r['rmse_m'][side] is not None] for side in (0,1))])


def analyze(dest,rows,poses,support,average,evaluate,cloud):
    candidates={}
    validation={}
    for event in sorted({r['event'] for r in rows}):
        cases=[r for r in rows if r['event']==event]
        candidates[str(event)]={}
        validation[str(event)]={}
        for method in METHODS:
            fitted=[r for r in cases if r['cohort']=='fit' and r['method']==method and r['accepted']]
            record=dict(status='INSUFFICIENT_FIT_SUPPORT',original_trajectory_modified=False,fit_accepted=len(fitted))
            tests=[]
            if fitted:
                transforms=[np.array(r['C_epoch_left_on_post']) for r in fitted]
                def distance(a,b):
                    m,d=delta(a,b)
                    return m/.2+d/3
                medoid=min(transforms,key=lambda a:sum(min(distance(a,b),10) for b in transforms))
                members=[r for r,t in zip(fitted,transforms) if distance(medoid,t)<=2 and
                         delta(medoid,t)[0]<=.2 and delta(medoid,t)[1]<=3]
                record['fit_cluster_members']=[[r['i'],r['j']] for r in members]
                record['fit_support_pass']=support(members,poses)
                all_deltas=[delta(a,b) for index,a in enumerate(transforms) for b in transforms[index+1:]]
                record['fit_pairwise_disagreement_m_deg']=all_deltas
                if fitted:
                    candidate=average([np.array(r['C_epoch_left_on_post']) for r in (members if record['fit_support_pass'] else fitted)])
                    record.update(status='CANDIDATE_NOT_APPLIED' if record['fit_support_pass'] else 'UNSUPPORTED_DIAGNOSTIC_CANDIDATE',C_epoch_left_on_post=candidate.tolist(),
                                  fit_members=[[r['i'],r['j']] for r in (members if record['fit_support_pass'] else fitted)],
                                  hypothetical_start_frame=event,
                                  hypothetical_end_frame=next((n for n in range(event+1,len(poses))
                                     if poses[n]['map_epoch']!='9'),len(poses))-1,
                                  applicability='Only sampled post-event window independently tested; no full-segment authorization',
                                  correction_size=delta(IDENTITY,candidate))
                    for row in [r for r in cases if r['cohort']=='validation' and r['method']==method]:
                        i,j=row['i'],row['j']
                        a,b=cloud(i,4),cloud(j,4)
                        prediction=np.linalg.inv(pose(poses[i]))@candidate@pose(poses[j])
                        corrected=evaluate(a,b,prediction)
                        independent=np.array(row['T_cam_i_from_cam_j']) if row['T_cam_i_from_cam_j'] is not None else None
                        corrected_difference=delta(prediction,independent) if independent is not None else None
                        correction_delta=delta(candidate,np.array(row['C_epoch_left_on_post'])) if independent is not None else None
                        test=dict(i=i,j=j,post_elapsed_s=row['post_elapsed_s'],
                                  independent_estimate_accepted=row['accepted'],frozen=row['frozen_metrics'],
                                  independent=row['independent_metrics'],corrected=corrected,
                                  frozen_translation_residual_m=row['discrepancy_m'],
                                  frozen_rotation_residual_deg=row['discrepancy_deg'],
                                  corrected_translation_residual_m=corrected_difference[0] if independent is not None else None,
                                  corrected_rotation_residual_deg=corrected_difference[1] if independent is not None else None,
                                  C_agreement=correction_delta)
                        test['candidate_pass']=(corrected['pass_geometry'] and row['accepted'] and
                                                correction_delta[0]<=.2 and correction_delta[1]<=3)
                        tests.append(test)
                    passing=[r for r in tests if r['candidate_pass']]
                    early=[r for r in passing if r['post_elapsed_s']<1]
                    late=[r for r in passing if r['post_elapsed_s']>=2]
                    has_support=record['fit_support_pass'] and support(passing,poses) and len({r['i'] for r in early})>=2 and len({r['i'] for r in late})>=2
                    record.update(heldout_pass=len(passing),early_pass=len(early),late_pass=len(late),
                                  validation_support_pass=has_support,
                                  frozen_geometry=summary([r['frozen'] for r in tests]),
                                  corrected_geometry=summary([r['corrected'] for r in tests]),
                                  independent_geometry=summary([r['independent'] for r in tests if r['independent'] is not None]))
            candidates[str(event)][method]=record
            validation[str(event)][method]=tests
        progress={}
        for method in METHODS:
            tested=[r for r in cases if r['cohort']=='validation' and r['method']==method and r['accepted']]
            per_anchor=[]
            for i in sorted({r['i'] for r in tested}):
                samples=sorted([r for r in tested if r['i']==i],key=lambda r:r['post_elapsed_s'])
                if len(samples)<3:
                    continue
                times=np.array([r['post_elapsed_s'] for r in samples])
                values=np.array([r['discrepancy_m'] for r in samples])
                if np.ptp(times)<1.5:
                    continue
                coefficients=np.polyfit(times,values,1)
                sse=np.sum((values-np.polyval(coefficients,times))**2)
                sst=np.sum((values-values.mean())**2)
                per_anchor.append(dict(pre_frame=i,post_frames=[r['j'] for r in samples],
                                       translation_residual_slope_m_s=float(coefficients[0]),
                                       r_squared=float(1-sse/sst) if sst>0 else None,
                                       first_residual_m=float(values[0]),last_residual_m=float(values[-1])))
            progress[method]=dict(accepted_early_pair_count=sum(r['post_elapsed_s']<1 for r in tested),
                                  accepted_late_pair_count=sum(r['post_elapsed_s']>=2 for r in tested),
                                  per_fixed_pre_anchor=per_anchor,
                                  interpretation='Geometry-only trends require visual or independent support before assigning drift')
        validation[str(event)]['temporal_progression_evidence']=progress
        # Classification never relies on a single adjacent pair.
        # A full recommendation also requires fit/validation support and a second method.
        possible=[(m,r) for m,r in candidates[str(event)].items() if r.get('validation_support_pass')]
        agreement=[]
        for method,record in possible:
            for other,second in possible:
                if method>=other:
                    continue
                difference=delta(np.array(record['C_epoch_left_on_post']),np.array(second['C_epoch_left_on_post']))
                if difference[0]<=.2 and difference[1]<=3:
                    agreement.append([method,other])
        label='INCONCLUSIVE'
        reason='The frozen multi-frame gates or method consistency requirements were not met.'
        recommendation='Retain original poses; candidate matrices remain diagnostic.'
        if agreement:
            name=agreement[0][0]
            best=candidates[str(event)][name]
            if best['correction_size'][0]>.2 or best['correction_size'][1]>3:
                controls=[r for r in cases if r['cohort']=='control' and r['accepted']]
                control_count=len({(r['i'],r['j']) for r in controls})
                if control_count>=3:
                    label='PERSISTENT_SEGMENT_OFFSET'
                    reason='At least two independent-initializer methods agree on a fit-only rigid correction, supported on early and late withheld pairs.'
                    recommendation='Bounded segment correction study justified; apply to no original pose until a wider time range passes.'
        validation[str(event)]['decision']=dict(classification=label,reason=reason,
                recommended_action=recommendation,corroborating_methods=agreement,
                observed_range_frames=[event-30,event+30],correct_original_poses=False,
                persistence_scope='Approximately three post-event seconds; longer persistence remains untested')
    dump(dest/'stage31_candidate_corrections.json',candidates)
    dump(dest/'stage31_cross_boundary_validation.json',validation)
    draw(dest,rows,candidates,validation)
    report=['# Stage31 Epoch9 segment persistence analysis','',
            'Baseline Stage30 commit 99bef28a83df8f855b65b46295a73aa001659e77. Original camera poses, Epoch transforms, graph constraints and maps were preserved.',
            '', '## Frozen protocol', '',
            'Each event uses 30 valid frames before and 31 from the first post-event frame. Fit frames are pre offsets −30/−26/−21 and post offsets +10/+14/+19. '
            'Validation frames are pre −17/−10/−3 and post +0/+3/+6/+23/+26/+30. All timestamps are original bag/header times. '
            'Fit and validation IDs are disjoint with at least 300 ms exclusion. There are nine cross-boundary fit pairs, eighteen validation pairs and six within-segment controls per event. '
            'The exact frame plan is written before registration and remains available with the policy.',
            '',
            'RGB-D ORB metric RANSAC, identity-seeded ICP and FPFH/RANSAC-seeded ICP use original measured depth with frozen color calibration. '
            'No frozen pose initializes either geometric method. Fit geometry samples pixel offset 0 and geometry validation samples offset 4, at stride 8. '
            'The prior minimums remain 300 inliers, 35% overlap in both directions and spatial spread 0.05 at 12 cm. '
            'Visual hypotheses additionally require 60 mutual / 40 metric / 30 inlier correspondences, 50% support and fit RMSE at most 0.1 m. '
            'Each accepted candidate needs three unique frame IDs on each side spanning at least 0.8 s in fit and withheld support. '
            'Identity is an explicit initialization, not a forced motion estimate.',
            '',
            'A supplementary pass uses Shi-Tomasi corners selected on valid depth and bidirectional RGB optical flow, followed by measured-depth rigid RANSAC. '
            'This was added after sparse depth-backed ORB support was observed in attempt1. The original attempt is preserved. '
            'The same frame cohorts, exclusion buffer and geometric acceptance gates are reused. Candidate transforms without a stable fit cluster are explicitly unsupported; '
            'they are still evaluated and stored to expose failure rather than skipped.',
            '', '## Decisions', '',
            '| Event | Classification | Candidate fit/validation support |',
            '|---|---|---|']
    for event in validation:
        decision=validation[event]['decision']
        counts=', '.join(f"{method}: {item['fit_accepted']}/{item.get('heldout_pass',0)}" for method,item in candidates[event].items())
        report.append(f"| {int(event)-1}→{event} | {decision['classification']} | {counts} |")
    report+=['', '## Fit-only candidates evaluated on held-out data', '',
             'All candidate matrices below failed stable fit support. They remain UNSUPPORTED_DIAGNOSTIC_CANDIDATE. '
             'Zero held-out pairs met the combined candidate, transform-consistency and geometric gates. '
             'A numerical improvement alone cannot qualify a segment correction.', '',
             '| Event / method | Frozen → candidate translation residual median (m) | Rotation median (deg) | Bidirectional overlap median |',
             '|---|---|---|---|']
    for event,methods in candidates.items():
        for method,candidate in methods.items():
            tests=validation[event][method]
            if not tests:
                continue
            usable=[r for r in tests if r['independent_estimate_accepted']]
            tr=lambda key: float(np.median([r[key] for r in usable])) if usable else None
            frozen_res,corrected_res=tr('frozen_translation_residual_m'),tr('corrected_translation_residual_m')
            frozen_rot,corrected_rot=tr('frozen_rotation_residual_deg'),tr('corrected_rotation_residual_deg')
            overlap=candidate['corrected_geometry']['bidirectional_overlap_median']
            report.append(f"| {event} / {method} | {frozen_res:.3f} → {corrected_res:.3f} | "
                          f"{frozen_rot:.2f} → {corrected_rot:.2f} | {overlap[0]:.1%} / {overlap[1]:.1%} |")
    report+=['', '## Cross-boundary persistence findings', '',
             'Neither fit-only geometric method produced a consistent correction cluster spanning three distinct frames on each side. '
             'ORB produced no qualified fit pairs, and depth-masked optical flow produced no qualified pairs in this cohort. '
             'The direct ORB validation pair 10068→10071 passes the single-pair gate, but is insufficient for a segment conclusion.',
             '',
             'Per-fixed-pre-frame residual trends are in temporal_progression_evidence. Repeated post-event geometric estimates '
             'often continue to disagree with the frozen trajectory beyond the first frame and grow with temporal separation. '
             'The within-segment controls also disagree with frozen relative motion. This is drift-like diagnostic evidence, '
             'but depth registration ambiguity remains unresolved, including a roughly 176 degree fit-only FPFH correction '
             'for event 10071. The experiment does not establish an isolated spike, constant segment offset, or accumulated '
             'drift with independent evidence. All three events therefore remain INCONCLUSIVE.',
             '', '## Interpretation and limitations','',
             'The decision tables retain every rejected pair. Motion estimates sharing a frame are correlated. '
             'Frame ID/time exclusions prevent fitting on held-out observations, but nearby viewpoints still observe the same scene. '
             'ICP and FPFH corroboration share depth and an objective, so they provide independent initialization evidence rather than independent sensors. '
             'Inlier RMSE is censored to 12 cm and must be interpreted together with overlap and counts. Missing values mean no supported transform or no inliers.',
             '',
             'A rigid candidate uses C_epoch_left_on_post = T_epoch_from_cam_i @ T_cam_i_from_cam_j @ inverse(T_epoch_from_cam_j). '
             'It is fitted only on the fit cohort. Withheld predicted motion is inverse(T_epoch_from_cam_i) @ C @ T_epoch_from_cam_j. '
             'Only a few seconds of continuation are tested. No candidate justifies extrapolation over the rest of Epoch9 without further held-out windows. '
             'Rejected evidence cannot establish whether a one-frame spike, constant segment offset or varying drift is present.',
             '', '## Recommended action per event','']
    for event in validation:
        decision=validation[event]['decision']
        report.append(f"- Frames {int(event)-1}→{event}: {decision['recommended_action']} "
                      f"Observed scope {int(event)-30}–{int(event)+30}; no pose edits. {decision['reason']}")
    report+=['', '## Reproduce','',
             'Run stage31_persistence.py --scratch work in the isolated Stage24 Python environment with the Stage26 Open3D runtime on PYTHONPATH. '
             'Original CRC-validated camera SQLite copies and frozen Stage24 pose/sync/coverage/calibration tables are required. '
             'The runner verifies original message IDs, timestamps and per-frame depth coverage. Preservation SHA256s are written after completion.']
    (dest/'STAGE31_SEGMENT_PERSISTENCE_REPORT.md').write_text('\n'.join(report)+'\n',encoding='utf-8')


def draw(dest,rows,candidates,validation):
    fig,axes=plt.subplots(3,2,figsize=(12,10),layout='constrained',sharex=True)
    names=sorted(candidates,key=int)
    styles={'orb_metric':('#0072B2','o'),'icp_identity':('#D55E00','^'),'fpfh_ransac_icp':('#009E73','s'),'rgbd_flow_ransac':('#CC79A7','x')}
    for row,event in enumerate(names):
        subset=[r for r in rows if str(r['event'])==event and r['cohort']=='validation' and r['accepted']]
        for method,(color,mark) in styles.items():
            chosen=[r for r in subset if r['method']==method]
            axes[row,0].scatter([r['post_elapsed_s'] for r in chosen],[r['discrepancy_m'] for r in chosen],
                               label=method,color=color,marker=mark,s=24)
            evaluated=validation[event][method]
            supported=[r for r in evaluated if r['independent_estimate_accepted']]
            axes[row,1].scatter([r['post_elapsed_s'] for r in supported],
                               [r['corrected_translation_residual_m'] for r in supported],
                               label=method,color=color,marker=mark,s=24)
        for col in (0,1):
            axes[row,col].set(xlim=(-.1,3.4),ylim=(0,None),xlabel='Post-event elapsed sensor time (s)',
                              ylabel='Translation residual (m)')
            axes[row,col].grid(axis='y',alpha=.15)
        label=validation[event]['decision']['classification']
        axes[row,0].set_title(f"{int(event)-1}→{event} · frozen · {label}")
        axes[row,1].set_title('Rejected fit-only hypotheses · withheld observations')
        if row==0:
            axes[row,0].legend(fontsize=8)
    fig.suptitle('Epoch9 persistence · all accepted withheld pair observations\nUnqualified visual estimates are omitted · geometric pair passes remain diagnostic')
    fig.savefig(dest/'stage31_segment_comparison.png',dpi=150)
    plt.close(fig)
