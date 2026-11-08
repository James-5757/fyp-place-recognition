"""Actual local browser interactions; no scientific SLAM rerun."""
import argparse,json,time
from pathlib import Path
from playwright.sync_api import sync_playwright

def run(args):
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True);errors=[];bad=[]
    with sync_playwright() as p:
        b=p.chromium.launch(executable_path=args.chromium,headless=True,args=['--use-angle=swiftshader','--enable-webgl'])
        page=b.new_page(viewport={'width':1600,'height':1120},device_scale_factor=1)
        page.on('pageerror',lambda e:errors.append(str(e)))
        page.on('response',lambda r:bad.append({'url':r.url,'status':r.status}) if r.status>=400 else None)
        start=time.perf_counter();page.goto(args.url,wait_until='networkidle',timeout=60000)
        page.wait_for_function("typeof window.rgbdDemoAudit==='function'",timeout=60000)
        startup=(time.perf_counter()-start)*1000
        audit=lambda:page.evaluate('window.rgbdDemoAudit()')
        first=audit();assert first['ready'] and first['index']==0 and first['geometry_builds']==first['expected_geometry_builds']
        assert first['merged_map'] is False and first['LiDAR'] is False
        page.screenshot(path=str(out/'start.png'),full_page=True)
        page.locator('#play').click();page.wait_for_timeout(1700);played=audit();assert played['index']>0
        page.locator('#play').click();paused=audit();assert not paused['playing']
        page.wait_for_timeout(400);assert audit()['index']==paused['index']
        page.locator('#seek').evaluate("x=>{x.value=30;x.dispatchEvent(new Event('input',{bubbles:true}));}")
        page.wait_for_function("Array.from(document.images).every(x=>x.complete&&x.naturalWidth>0)")
        middle=audit();assert middle['index']==30
        page.screenshot(path=str(out/'rgbd_building.png'),full_page=True)
        page.locator('#final').click();page.wait_for_function("Array.from(document.images).every(x=>x.complete&&x.naturalWidth>0)")
        final=audit();assert final['index']==59
        for r in ['robot1','robot3']:assert final['robots'][r]['points_visible']==final['robots'][r]['points_total']
        page.screenshot(path=str(out/'final_local_maps.png'),full_page=True)
        source=page.evaluate("fetch('data.json').then(r=>r.json())")
        for i in range(60):
            page.locator('#seek').evaluate("(x,i)=>{x.value=i;x.dispatchEvent(new Event('input',{bubbles:true}));}",i)
            state=audit()
            for robot,s in source['robots'].items():
                actual=state['robots'][robot];expected=s['frames'][i]
                assert actual['active_epoch']==expected['tracking_epoch'] and actual['visible_submaps']==1
                assert actual['points_visible']==expected['points_end']
        page.locator('#final').click();page.wait_for_timeout(1500)
        started=time.perf_counter();n=audit()['render_frames'];page.wait_for_timeout(3000)
        fps=(audit()['render_frames']-n)/(time.perf_counter()-started)
        interactions={}
        for robot in ['robot1','robot3']:
            box=page.locator('#'+robot+' .map').bounding_box();x=box['x']+box['width']*.5;y=box['y']+box['height']*.5
            page.wait_for_timeout(1500);before=audit()['robots'][robot]
            page.mouse.move(x,y);page.mouse.down();page.mouse.move(x+75,y+40,steps=10);page.mouse.up();page.wait_for_timeout(1500)
            orbit=audit()['robots'][robot];assert sum(abs(a-b) for a,b in zip(before['camera'],orbit['camera']))>.01
            page.mouse.move(x,y);page.mouse.wheel(0,-220);page.wait_for_timeout(1500)
            zoom=audit()['robots'][robot];assert sum(abs(a-b) for a,b in zip(orbit['camera'],zoom['camera']))>.01
            page.mouse.move(x,y);page.mouse.down(button='right');page.mouse.move(x+45,y+15,steps=10);page.mouse.up(button='right');page.wait_for_timeout(1500)
            pan=audit()['robots'][robot];assert sum(abs(a-b) for a,b in zip(zoom['target'],pan['target']))>.01
            interactions[robot]={'orbit':'PASS','zoom':'PASS','pan':'PASS'}
        assert audit()['geometry_builds']==first['geometry_builds']
        page.locator('#reset').click();assert audit()['index']==0
        assert not errors and not bad,(errors,bad)
        report=dict(status='PASS',startup_ms=startup,renderer='headless Chromium software WebGL; not a hardware performance guarantee',
            play='PASS',pause='PASS',seek='PASS',reset='PASS',final='PASS',image_decode='PASS',
            all_60_frame_epoch_and_no_future_point_checks='PASS',measured_software_render_fps=fps,
            interactions=interactions,geometry_builds=first['geometry_builds'],initial=first,middle=middle,final_state=final,
            console_errors=errors,http_errors=bad)
        (out/'browser_validation.json').write_text(json.dumps(report,indent=2)+'\n',newline='\n');print(json.dumps(report))
        b.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--url',default='http://127.0.0.1:8021/rgbd_camera_mapping/');p.add_argument('--out',required=True);p.add_argument('--chromium',required=True);run(p.parse_args())
