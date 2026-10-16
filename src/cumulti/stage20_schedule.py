"""Pure deterministic dual-stream availability scheduler; no GT or solver."""
def schedule(streams, loops):
    events=[]
    for robot,rows in streams.items():
        origin=int(rows[0][1]);previous=None
        for k,t in rows:
            assert previous is None or int(t)>previous
            events.append(dict(robot_id=robot,keyframe_id=int(k),timestamp_ns=int(t),normalized_ns=int(t)-origin))
            previous=int(t)
    events.sort(key=lambda e:(e['normalized_ns'],e['robot_id'],e['keyframe_id']))
    pos={}
    for i,e in enumerate(events):e['event_index']=i;pos[(e['robot_id'],e['keyframe_id'])]=i
    assert len(pos)==len(events)
    eligible=[]
    for loop in loops:
        q=pos[(loop['query_robot'],loop['query_keyframe_id'])];c=pos[(loop['candidate_robot'],loop['candidate_keyframe_id'])]
        j=max(q,c)
        eligible.append(dict(loop,query_event_index=q,candidate_event_index=c,release_event_index=j,query_normalized_ns=events[q]['normalized_ns'],candidate_normalized_ns=events[c]['normalized_ns'],eligibility_ns=max(events[q]['normalized_ns'],events[c]['normalized_ns'])))
    eligible.sort(key=lambda l:(l['release_event_index'],l['frozen_loop_id']))
    assert len({l['frozen_loop_id'] for l in eligible})==len(eligible)
    for i,l in enumerate(eligible,1):l['eligibility_order']=i
    return events,eligible

def replay_registry(events,loops):
    """Independent logical replay, useful for both unit tests and saved-trace audits."""
    nodes=set();inserted=set();rows=[];odom=set();releases={}
    for l in loops:releases.setdefault(l['release_event_index'],[]).append(l)
    starts={e['robot_id']:e['keyframe_id'] for e in reversed(events)}
    for e in events:
        n=(e['robot_id'],e['keyframe_id']);assert n not in nodes;nodes.add(n)
        if n[1]!=starts[n[0]]:
            assert (n[0],n[1]-1) in nodes;odom.add((n[0],n[1]-1,n[1]))
        released=[]
        for l in releases.get(e['event_index'],[]):
            assert (l['query_robot'],l['query_keyframe_id']) in nodes
            assert (l['candidate_robot'],l['candidate_keyframe_id']) in nodes
            assert l['frozen_loop_id'] not in inserted
            inserted.add(l['frozen_loop_id']);released.append(l['frozen_loop_id'])
        rows.append(dict(event_index=e['event_index'],nodes=len(nodes),odometry=len(odom),loops=len(inserted),released=released,components=1 if inserted else len({r for r,k in nodes})))
    return rows
