"""Forensic plots generated directly from measured Stage30 evidence."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def draw(dest,steps,jumps,representatives,edges):
    fig=plt.figure(figsize=(12.8,10),layout='constrained')
    grid=fig.add_gridspec(4,4,height_ratios=[1,1,1.35,1.35])
    timeline=[fig.add_subplot(grid[i,:]) for i in range(2)]
    for ax,key,label,threshold in zip(timeline,('m','deg'),('Translation step (m)','Rotation step (°)'),(1,35)):
        for epoch,color,marker in ((6,'#0072B2','o'),(9,'#D55E00','+')):
            values=[r for r in steps if r['epoch']==epoch]
            ax.scatter([v['t_s'] for v in values],[v[key] for v in values],s=7,c=color,marker=marker,label=f'Epoch {epoch}')
        ax.axhline(threshold,ls='--',color='#555555',lw=.8,label='Audit threshold')
        ax.set(xlabel='Original elapsed camera time (s)',ylabel=label)
    timeline[0].legend(ncols=3)
    for row,entry in enumerate(representatives):
        for n,index in enumerate(entry['ids']):
            for group in (0,1):
                ax=fig.add_subplot(grid[row+2,n+2*group])
                if group==0:
                    ax.imshow(entry['rgb'][n],interpolation='nearest')
                else:
                    depth=np.ma.masked_where(entry['depth'][n]==0,entry['depth'][n]/1000)
                    norm=plt.get_cmap('viridis').with_extremes(bad='#303030')
                    ax.imshow(depth,vmin=.3,vmax=10,cmap=norm,interpolation='nearest')
                    if not depth.count():
                        ax.text(.5,.5,'No synchronized depth',transform=ax.transAxes,color='white',ha='center')
                ax.set_title(f"E{entry['epoch']} frame {index} · "+('RGB' if not group else 'Depth (0.3–10 m)'))
                ax.set_axis_off()
    fig.suptitle('Robot3 native RGB-D pose discontinuities')
    fig.savefig(dest/'stage30_pose_discontinuity.png',dpi=150)
    plt.close(fig)

    fig,axes=plt.subplots(2,1,figsize=(10,7),layout='constrained')
    slots=[]
    for index,(edge,evidence) in enumerate(edges['edges'].items()):
        slots.append(edge)
        for name,x,color,mark in (('frozen',index-.13,'#0072B2','o'),('composed_chain',index+.13,'#D55E00','^'),
                                  ('closure_implied_single_edge',index+.13,'#009E73','s')):
            measurements=[r[name] for r in evidence['heldout'] if name in r]
            for j,dirs in enumerate(measurements):
                point=x + (j-(len(measurements)-1)/2)*.033
                for side,result in enumerate(dirs):
                    axes[0].scatter(point,100*result['overlap'],marker=mark,s=35,edgecolors=color,
                                    facecolors=color if side==0 else 'none',label=name if j==side==0 and
                                    (index==0 or name=='composed_chain') else None)
                    if result['rmse_m'] is not None:
                        axes[1].scatter(point,result['rmse_m'],marker=mark,s=35,edgecolors=color,
                                        facecolors=color if side==0 else 'none')
    axes[0].axhline(35,ls='--',color='#555555',lw=.9,label='Stage26 overlap criterion')
    axes[0].set(ylabel='Directional overlap (%)',ylim=(0,100))
    axes[1].set(ylabel='Inlier RMSE (m)',ylim=(0,.125),xlabel='Epoch placement tested on held-out frame pairs')
    for ax in axes:
        ax.set_xticks(range(len(slots)),[x.replace('->',' → ') for x in slots])
    handles,labels=axes[0].get_legend_handles_labels()
    axes[0].legend(handles,['Frozen per-edge placement' if x=='frozen' else '16→17→18→19 composition' if
                            x=='composed_chain' else 'Closure implied by replacing one edge' if x=='closure_implied_single_edge' else x for x in labels],ncols=2)
    fig.suptitle('Independent Robot3 geometry checks · 12 cm correspondence gate\nFilled: later→earlier camera; open: earlier→later; missing RMSE: zero inliers')
    fig.savefig(dest/'stage30_registration_comparison.png',dpi=150)
    plt.close(fig)
