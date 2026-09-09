import argparse
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image

ALPHA = 0.6
FUSION = "sc_cross_max_alpha_0.6"
OFFSETS = [-10, -5, 0]


def panel_frames(frame, rgb):
    return [(o, rgb / f"{frame + o:06d}.png") for o in OFFSETS if frame + o >= 0 and (rgb / f"{frame + o:06d}.png").is_file()]


def make_panel(query, a, b, best_q, best_c, path, rgb):
    fig, axes = plt.subplots(3, 3, figsize=(15, 7), squeeze=False)
    for ri, (label, frame) in enumerate([("QUERY", query), ("CANDIDATE A", a), ("CANDIDATE B", b)]):
        sources = panel_frames(frame, rgb)
        for ci, (offset, image_path) in enumerate(sources):
            ax = axes[ri][ci]
            with Image.open(image_path) as image: ax.imshow(image.convert("RGB"))
            ax.set_axis_off()
            if ri == 0: ax.set_title("t" if offset == 0 else f"t{offset}", fontsize=10)
            if ci == 0: ax.text(-0.035, .5, label, transform=ax.transAxes, ha="right", va="center", fontsize=11)
            if (ri == 0 and frame + offset == best_q) or (ri > 0 and frame + offset == best_c):
                for spine in ax.spines.values(): spine.set_visible(True); spine.set_edgecolor("gold"); spine.set_linewidth(3)
        for ci in range(len(sources), 3): axes[ri][ci].set_axis_off()
    fig.suptitle("Temporal RGB Place Verification", fontsize=14, fontweight="bold")
    fig.text(.5, .015, "Highlighted frames show one automatically selected visual correspondence.", ha="center", fontsize=9)
    fig.tight_layout(rect=(.08, .04, 1, .94)); fig.savefig(path, dpi=150, bbox_inches="tight"); plt.close(fig)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--scores", type=Path, default=Path("outputs/top20_visual_filtering/candidate_scores.csv"))
    p.add_argument("--rgb-dir", type=Path, default=Path("data/kitti/dataset/sequences/00/image_2"))
    p.add_argument("--output-dir", type=Path, default=Path("outputs/selective_vlm_v2"))
    p.add_argument("--overwrite", action="store_true")
    a = p.parse_args()
    if a.output_dir.exists() and not a.overwrite: raise FileExistsError(a.output_dir)
    scored = pd.read_csv(a.scores); top5=[]; status=[]; disagreements=[]; comparison=[]; controls=[]
    for q, g in scored.groupby("query_frame", sort=True):
        fused = g.sort_values([FUSION, "sc_rank"], ascending=[False, True]).head(5).copy(); fused["fusion_rank"] = range(1,6); top5.append(fused)
        anchor = g.sort_values("sc_rank").iloc[0]; anchor_id = int(anchor.candidate_frame)
        status.append({"query_frame":int(q),"pool_status":"POOL_CONTAINS_POSITIVE" if fused.is_positive.astype(int).any() else "FILTERED_POOL_MISS","filtered_top5_contains_positive":bool(fused.is_positive.astype(int).any())})
        fusion_top=fused.iloc[0]; cross_top=fused.sort_values(["temporal_cross_max","sc_rank"],ascending=[False,True]).iloc[0]
        nonanchor=fused[fused.candidate_frame.astype(int)!=anchor_id]
        fchall=nonanchor.sort_values("fusion_rank").iloc[0]; cchall=nonanchor.sort_values(["temporal_cross_max","sc_rank"],ascending=[False,True]).iloc[0]
        challenger_ids={int(fchall.candidate_frame),int(cchall.candidate_frame)}
        if anchor_id != int(fusion_top.candidate_frame) or anchor_id != int(cross_top.candidate_frame) or len(challenger_ids)==2:
            disagreements.append({"query_frame":int(q),"sc_rank1_frame":anchor_id,"fusion_rank1_frame":int(fusion_top.candidate_frame),"cross_max_rank1_frame":int(cross_top.candidate_frame),"best_fusion_challenger":int(fchall.candidate_frame),"best_crossmax_challenger":int(cchall.candidate_frame)})
            for c in challenger_ids: comparison.append((int(q),"DISAGREEMENT",anchor_id,c))
        if int(anchor.is_positive)==1 and anchor_id==int(fusion_top.candidate_frame):
            ordered=g.sort_values("sc_rank"); controls.append({"query_frame":int(q),"sc_margin":float(ordered.iloc[0].scan_context_score-ordered.iloc[1].scan_context_score)})
    filtered=pd.concat(top5,ignore_index=True); pool=pd.DataFrame(status); controls=pd.DataFrame(controls)
    low=controls.sort_values(["sc_margin","query_frame"]).head(10).assign(case_type="LOW_MARGIN_CONTROL")
    high=controls.sort_values(["sc_margin","query_frame"],ascending=[False,True]).head(10).assign(case_type="HIGH_MARGIN_CONTROL")
    controls=pd.concat([low,high],ignore_index=True)
    for r in controls.itertuples(index=False):
        g=filtered[filtered.query_frame==r.query_frame]; anchor=int(g.sort_values("sc_rank").iloc[0].candidate_frame); challenger=int(g[g.candidate_frame.astype(int)!=anchor].sort_values("fusion_rank").iloc[0].candidate_frame); comparison.append((int(r.query_frame),r.case_type,anchor,challenger))
    comparison=sorted(set(comparison),key=lambda x:(x[1],x[0],x[2],x[3])); collage_dir=a.output_dir/"collages"; collage_dir.mkdir(parents=True,exist_ok=True); manifest=[]
    for q,kind,anchor,challenger in comparison:
        crow=filtered[(filtered.query_frame==q)&(filtered.candidate_frame==challenger)].iloc[0]; arow=filtered[(filtered.query_frame==q)&(filtered.candidate_frame==anchor)].iloc[0]
        for order,A,B in [("AB",anchor,challenger),("BA",challenger,anchor)]:
            path=collage_dir/f"q{q:06d}_{kind.lower()}_a{anchor:06d}_b{challenger:06d}_{order}.png"; make_panel(q,A,B,int(crow.best_query_temporal_frame),int(crow.best_candidate_temporal_frame),path,a.rgb_dir)
            rowA=filtered[(filtered.query_frame==q)&(filtered.candidate_frame==A)].iloc[0];rowB=filtered[(filtered.query_frame==q)&(filtered.candidate_frame==B)].iloc[0]
            manifest.append({"query_frame":q,"case_type":kind,"anchor_frame":anchor,"challenger_frame":challenger,"order":order,"candidate_A_frame":A,"candidate_B_frame":B,"collage_path":str(path),"anchor_is_positive":int(arow.is_positive),"challenger_is_positive":int(crow.is_positive),"candidate_A_is_positive":int(rowA.is_positive),"candidate_B_is_positive":int(rowB.is_positive),"pool_contains_positive":bool(pool[pool.query_frame==q].iloc[0].filtered_top5_contains_positive)})
    a.output_dir.mkdir(parents=True,exist_ok=True); filtered.to_csv(a.output_dir/"filtered_top5.csv",index=False);pool.to_csv(a.output_dir/"pool_status.csv",index=False);pd.DataFrame(disagreements).to_csv(a.output_dir/"disagreement_queries.csv",index=False);controls.to_csv(a.output_dir/"control_queries.csv",index=False);pd.DataFrame(manifest).to_csv(a.output_dir/"manifest.csv",index=False)
    retention=pool.filtered_top5_contains_positive.mean()
    if len(filtered)!=790 or filtered.query_frame.nunique()!=158 or abs(retention-157/158)>1e-12: raise RuntimeError(f"sanity failed {len(filtered)} {filtered.query_frame.nunique()} {retention}")
    print("filtered_top5",len(filtered),"retention",retention,"pool_misses",int((~pool.filtered_top5_contains_positive).sum()),"disagreements",len(disagreements),"controls",len(controls),"comparisons",len(comparison),"requests",len(manifest))

if __name__=="__main__": main()
