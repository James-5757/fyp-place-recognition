from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT=Path('outputs/report_figures'); OUT.mkdir(parents=True,exist_ok=True)
BG='#F7F4EE';INK='#172631';MUTED='#5B6873';BLUE='#284B63';TEAL='#447A72';RED='#D45D3D';GOLD='#C9923D';GRID='#D8D3CA'
plt.rcParams.update({'font.family':'DejaVu Sans','figure.facecolor':BG,'axes.facecolor':BG,'savefig.facecolor':BG,'axes.titleweight':'bold'})

def finish(fig,path):
    fig.savefig(OUT/path,dpi=220,bbox_inches='tight',facecolor=BG); plt.close(fig)

# Method comparison: labels and annotation have dedicated margins.
methods=pd.DataFrame([
 ('Scan Context',151,158),('BEV OpenCLIP',145,158),('Single-frame RGB',141,158),
 ('Temporal Mean-5',135,158),('Cross-Frame Max',147,158),('SC + Cross-Max (alpha=0.7)',153,158)])
methods.columns=['method','hit','total']; methods['r1']=100*methods.hit/methods.total
fig,ax=plt.subplots(figsize=(12,7.3)); y=np.arange(len(methods)); cols=[BLUE,'#91A2B2','#B94D69',GOLD,TEAL,RED]
ax.hlines(y,80,methods.r1,color=cols,lw=5,alpha=.9); ax.scatter(methods.r1,y,s=260,color=cols,edgecolor=INK,lw=1.5,zorder=3)
for yi,row in zip(y,methods.itertuples()):
    ax.text(100.65,yi,f'{row.r1:.2f}%  ({row.hit}/{row.total})',va='center',ha='right',fontsize=12,color=INK)
ax.set_yticks(y);ax.set_yticklabels(methods.method,fontsize=12);ax.invert_yaxis();ax.set_xlim(80,100.8);ax.set_xticks([80,85,90,95,100]);ax.set_xlabel('R@1 (%)',fontsize=13,fontweight='bold');ax.grid(axis='x',color=GRID,lw=.8);ax.spines[['top','right','left']].set_visible(False);ax.tick_params(axis='y',length=0)
ax.axvline(methods.iloc[0].r1,color=BLUE,ls='--',lw=1.5,alpha=.8)
ax.text(methods.iloc[0].r1+.05,-.68,'SC baseline',color=BLUE,fontsize=10,ha='left')
ax.annotate('+1.27 pp vs SC',xy=(methods.iloc[5].r1,y[5]),xytext=(92.1,4.55),arrowprops=dict(arrowstyle='->',lw=2,color=RED),color=RED,fontsize=12,fontweight='bold',ha='left')
fig.suptitle('Place Recognition Performance Across Visual Verification Stages',x=.08,y=.985,ha='left',fontsize=18,color=INK)
fig.text(.08,.945,'KITTI Sequence 00 · formal split · final Top-20 SC + Cross-Max fusion comparison',fontsize=10,color=MUTED)
fig.subplots_adjust(left=.27,right=.95,top=.84,bottom=.14);finish(fig,'method_r1_lollipop.png')

# Viewpoint motivation with non-overlapping text columns.
fig,axs=plt.subplots(1,2,figsize=(13,6.8),gridspec_kw={'width_ratios':[.9,1.35]})
ax=axs[0];ax.set_xlim(0,1);ax.set_ylim(0,1);ax.axis('off')
box=FancyBboxPatch((.13,.39),.74,.28,boxstyle='round,pad=.02',fc='#E5F0EE',ec=TEAL,lw=2);ax.add_patch(box);ax.text(.5,.56,'SAME PHYSICAL PLACE',ha='center',va='center',fontsize=17,fontweight='bold',color='#28635B');ax.text(.5,.47,'spatial distance: 1.7–3.2 m',ha='center',fontsize=12,color=INK)
ax.add_patch(FancyArrowPatch((.25,.30),(.75,.30),arrowstyle='-|>',mutation_scale=18,lw=3,color=BLUE));ax.add_patch(FancyArrowPatch((.75,.20),(.25,.20),arrowstyle='-|>',mutation_scale=18,lw=3,color='#B94D69'));ax.text(.5,.34,'Query camera  → →',ha='center',fontsize=11,color=BLUE);ax.text(.5,.14,'Candidate camera  ← ←',ha='center',fontsize=11,color='#B94D69');ax.text(.5,.04,'GT heading difference: 128°–172°',ha='center',fontsize=12,fontweight='bold',color='#B24C35')
ax=axs[1];ax.axis('off');ax.set_xlim(0,1);ax.set_ylim(0,1)
steps=[('Single RGB','highly viewpoint-sensitive', '#B94D69'),('Temporal Mean Pooling','may dilute distinctive observations',GOLD),('Cross-Frame Matching','searches compatible observations across time',TEAL),('Cross-Max','lets strong local evidence dominate',BLUE)]
ys=[.82,.60,.38,.16]
for (title,desc,col),yy in zip(steps,ys):
 ax.add_patch(FancyBboxPatch((.03,yy-.075),.94,.15,boxstyle='round,pad=.015',fc='white',ec=col,lw=2));ax.text(.08,yy+.018,title,fontsize=13,fontweight='bold',color=col,ha='left');ax.text(.08,yy-.028,desc,fontsize=11,color=INK,ha='left')
 if yy!=ys[-1]: ax.add_patch(FancyArrowPatch((.5,yy-.08),(.5,yy-.135),arrowstyle='-|>',mutation_scale=13,lw=1.5,color='#89959C'))
fig.suptitle('Why Cross-Frame Matching Instead of Single-Frame RGB?',x=.06,y=.98,ha='left',fontsize=18,color=INK);fig.text(.06,.935,'Forward-facing cameras can observe the same place from near-opposite headings.',fontsize=10,color=MUTED);fig.text(.06,.02,'Spatial proximity does not guarantee visual overlap for forward-facing cameras.',fontsize=11,fontweight='bold',color=INK);fig.subplots_adjust(left=.04,right=.98,top=.86,bottom=.08,wspace=.12);finish(fig,'viewpoint_crossframe_motivation.png')

# Recall curve.
rec=pd.read_csv('outputs/sc_candidate_recall_analysis/recall_curve.csv');fig,ax=plt.subplots(figsize=(10,6));ax.plot(rec.k,100*rec.recall_at_k,color=TEAL,lw=3,marker='o',ms=9);
for row in rec.itertuples(): ax.annotate(f'{100*row.recall_at_k:.2f}%',(row.k,100*row.recall_at_k),xytext=(0,10),textcoords='offset points',ha='center',fontsize=10,color=INK)
ax.set_xticks(rec.k);ax.set_ylim(94.5,100.7);ax.set_xlabel('Candidate pool size K',fontweight='bold',fontsize=12);ax.set_ylabel('Candidate Recall (%)',fontweight='bold',fontsize=12);ax.grid(color=GRID);ax.spines[['top','right']].set_visible(False);fig.suptitle('Can a Higher-Recall Pool Break the Candidate Ceiling?',x=.08,y=.98,ha='left',fontsize=17);fig.text(.08,.93,'All 158 valid queries contain a positive candidate by Scan Context Rank 20.',fontsize=10,color=MUTED);fig.subplots_adjust(left=.12,right=.96,top=.82,bottom=.14);finish(fig,'candidate_recall_at_k.png')

# Candidate ceiling funnel.
fig,ax=plt.subplots(figsize=(10,7));ax.axis('off');ax.set_xlim(0,1);ax.set_ylim(0,1)
levels=[('158 valid queries',.84,.76,BLUE),('SC Top-5 contains GT\n154 / 158 · 97.47%',.62,.58,TEAL),('SC + Cross-Max Top-5\n157 / 158 · 99.37%',.40,.40,GOLD),('Remaining system misses\n1 filtering miss + 5 ranking misses',.18,.24,RED)]
for text,yy,w,col in levels:
 x=(1-w)/2;ax.add_patch(FancyBboxPatch((x,yy-.07),w,.14,boxstyle='round,pad=.015',fc='white',ec=col,lw=2.5));ax.text(.5,yy,text,ha='center',va='center',fontsize=14,fontweight='bold',color=INK)
for i in range(len(levels)-1): ax.add_patch(FancyArrowPatch((.5,levels[i][1]-.08),(.5,levels[i+1][1]+.08),arrowstyle='-|>',mutation_scale=15,lw=1.8,color='#89959C'))
ax.text(.13,.51,'4 retrieval misses\nfrom SC Top-5',ha='center',va='center',fontsize=11,color=RED);ax.text(.87,.29,'candidate filtering\nretains 157/158',ha='center',va='center',fontsize=11,color=TEAL)
fig.suptitle('The New Bottleneck: Candidate Recall Ceiling',x=.08,y=.97,ha='left',fontsize=18);fig.text(.08,.92,'The Top-20 SC pool is complete; downstream filtering must preserve that coverage.',fontsize=10,color=MUTED);fig.subplots_adjust(top=.86,bottom=.05,left=.03,right=.97);finish(fig,'candidate_recall_ceiling_funnel.png')

# GT rank, transition and trajectory regenerated with safer spacing.
miss=pd.read_csv('outputs/sc_candidate_recall_analysis/top5_miss_analysis.csv').sort_values('first_positive_rank',ascending=False).reset_index(drop=True)
fig,ax=plt.subplots(figsize=(11,6.4));colors=[RED,GOLD,TEAL,BLUE];y=np.arange(len(miss))
for i,row in miss.iterrows(): ax.hlines(i,1,row.first_positive_rank,color=colors[i],lw=4);ax.scatter(row.first_positive_rank,i,s=170,color=colors[i],edgecolor=INK,lw=1.2,zorder=3);ax.text(row.first_positive_rank+.35,i,f'Rank {int(row.first_positive_rank)}  ·  GT {row.first_positive_gt_distance:.2f} m',va='center',fontsize=11)
ax.set_yticks(y);ax.set_yticklabels([f'Query {int(q):04d}' for q in miss.query_frame],fontsize=12);ax.set_xlim(.5,20.8);ax.set_xticks([1,5,10,15,20]);ax.set_xlabel('First GT-positive rank in full SC ranking',fontweight='bold');ax.grid(axis='x',color=GRID);ax.spines[['top','right','left']].set_visible(False);ax.tick_params(axis='y',length=0);ax.axvline(5,color='#B24C63',ls='--');ax.axvline(10,color=GOLD,ls='--');ax.axvline(20,color=TEAL,ls='--');fig.suptitle('Where Do the Four SC Top-5 Misses First Recover?',x=.08,y=.98,ha='left',fontsize=17);fig.text(.08,.93,'All four positives appear by Rank 15; Top-20 achieves 100% candidate recall.',fontsize=10,color=MUTED);fig.subplots_adjust(left=.18,right=.96,top=.84,bottom=.14);finish(fig,'top5_miss_gt_rank_lollipop.png')

scores=pd.read_csv('outputs/top20_visual_filtering/candidate_scores.csv');trans=[]
for q,g in scores.groupby('query_frame'):
 sc=g.sort_values(['scan_context_score','sc_rank'],ascending=[False,True]).iloc[0];fu=g.sort_values(['sc_cross_max_alpha_0.7','sc_rank'],ascending=[False,True]).iloc[0];trans.append((int(sc.is_positive),int(fu.is_positive)))
mat=np.array([[trans.count((1,1)),trans.count((1,0))],[trans.count((0,1)),trans.count((0,0))]])
fig,ax=plt.subplots(figsize=(8,7));im=ax.imshow(mat,cmap='YlGnBu',vmin=0,vmax=151);ax.set_xticks([0,1],['Correct','Wrong']);ax.set_yticks([0,1],['Correct','Wrong']);ax.set_xlabel('SC + Cross-Max α=0.7',fontweight='bold');ax.set_ylabel('Scan Context baseline',fontweight='bold');
for i in range(2):
 for j in range(2): ax.text(j,i,str(mat[i,j]),ha='center',va='center',fontsize=28,fontweight='bold',color='white' if mat[i,j]>70 else INK)
fig.colorbar(im,ax=ax,fraction=.046,pad=.04,label='Queries');fig.suptitle('Error Transition Matrix: SC vs Top-20 SC + Cross-Max',x=.08,y=.98,ha='left',fontsize=16);fig.text(.5,.035,'2 corrections · 0 regressions · 153 / 158 correct',ha='center',fontsize=12,fontweight='bold',color='#28635B');fig.subplots_adjust(left=.16,right=.88,top=.84,bottom=.16);finish(fig,'sc_to_crossmax_transition_matrix.png')

poses=np.loadtxt('data/kitti/dataset/poses/00.txt').reshape(-1,3,4)[:,:,3];traj=poses[:,[0,2]];top20=pd.read_csv('outputs/top20_visual_filtering/sc_top20_candidates.csv');fig,axes=plt.subplots(2,2,figsize=(14,11),squeeze=False)
for ax,row in zip(axes.flat,miss.sort_values('query_frame').itertuples(index=False)):
 q=int(row.query_frame);pos=int(row.first_positive_frame);false=int(row.sc_rank1_frame);top5=top20[(top20.query_frame==q)&(top20.sc_rank<=5)].candidate_frame.astype(int).tolist();ax.plot(traj[:,0],traj[:,1],color='#C9CED2',lw=1);ax.scatter(traj[top5,0],traj[top5,1],s=32,color='#9EA8B1',label='SC Top-5');ax.scatter(*traj[q],s=100,color=BLUE,edgecolor='white',label='Query');ax.scatter(*traj[pos],s=120,color=TEAL,marker='*',edgecolor='white',label='First GT positive');ax.scatter(*traj[false],s=90,color=RED,marker='X',edgecolor='white',label='SC Rank-1 false');pts=traj[[q,pos,false]+top5];pad=25;ax.set_xlim(pts[:,0].min()-pad,pts[:,0].max()+pad);ax.set_ylim(pts[:,1].min()-pad,pts[:,1].max()+pad);ax.set_aspect('equal',adjustable='box');ax.set_title(f'Query {q:04d} · GT rank {int(row.first_positive_rank)}',loc='left',fontsize=13);ax.set_xlabel('World X (m)');ax.set_ylabel('World Z (m)');ax.grid(color='#E1DED7',lw=.6);ax.legend(loc='upper right',fontsize=8,framealpha=.9)
fig.suptitle('Trajectory Context of the Four SC Top-5 Retrieval Misses',fontsize=18,fontweight='bold',y=.985);fig.text(.5,.018,'Blue = query · green star = first GT positive · red X = SC Rank-1 false · grey = SC Top-5',ha='center',fontsize=10);fig.subplots_adjust(left=.07,right=.98,top=.91,bottom=.07,hspace=.32,wspace=.22);finish(fig,'top5_miss_trajectory_cases.png')
print('report figures regenerated')
