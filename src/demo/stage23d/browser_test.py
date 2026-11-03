"""Local real-browser smoke/interaction evidence, not SLAM performance."""
import argparse,json,time,hashlib
from pathlib import Path
from playwright.sync_api import sync_playwright

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--url',default='http://127.0.0.1:8024/stage23d_policy_map_compare/');ap.add_argument('--output',required=True);ap.add_argument('--executable');a=ap.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    with sync_playwright() as p:
        args=dict(headless=True,args=['--use-angle=swiftshader','--enable-webgl','--no-sandbox'])
        if a.executable:args['executable_path']=a.executable
        browser=p.chromium.launch(**args);page=browser.new_page(viewport={'width':1680,'height':1100},device_scale_factor=1);errors=[];failed=[];page.on('pageerror',lambda e:errors.append(str(e)));page.on('response',lambda r:failed.append([r.url,r.status]) if r.status>=400 else None)
        start=time.monotonic();page.goto(a.url);page.wait_for_function('window.stage23d?.ready === true',timeout=60000);page.wait_for_timeout(1000);checks={}
        checks['actual_file_loading']=page.evaluate('stage23d.metrics.loadedAssets===4 && stage23d.metrics.errors.length===0')
        checks['side_by_side_identity']=page.evaluate('stage23d.panels.map(p=>p.label).join() === "P0,P5" && stage23d.panels.every(p=>p.current.state.loops.length===(p.label==="P0"?288:231))')
        page.screenshot(path=str(out/'final_incremental.png'),full_page=True)
        page.mouse.move(380,400);page.mouse.down();page.mouse.move(490,470,steps=8);page.mouse.up();page.wait_for_timeout(400)
        checks['camera_rotation_synchronization']=page.evaluate('stage23d.panels[0].camera.position.distanceTo(stage23d.panels[1].camera.position)<1e-7 && stage23d.panels[0].controls.target.distanceTo(stage23d.panels[1].controls.target)<1e-7')
        page.mouse.wheel(0,-120);page.wait_for_timeout(400);checks['camera_zoom_synchronization']=page.evaluate('stage23d.panels[0].camera.position.distanceTo(stage23d.panels[1].camera.position)<1e-7')
        page.mouse.move(400,430);page.mouse.down(button='right');page.mouse.move(440,450,steps=5);page.mouse.up(button='right');page.wait_for_timeout(400);checks['camera_pan_synchronization']=page.evaluate('stage23d.panels[0].controls.target.distanceTo(stage23d.panels[1].controls.target)<1e-7')
        page.select_option('#overlay','both');checks['overlay_toggle']=page.evaluate('stage23d.panels.every(p=>p.current.other.robot1.visible&&p.current.other.robot3.visible)')
        page.select_option('#identity','P0_ONLY');checks['loop_identity_toggle']=page.evaluate('stage23d.panels.every(p=>p.current.loops.P0_ONLY.visible&&!p.current.loops.COMMON.visible&&!p.current.loops.P5_ONLY.visible)')
        region_names=page.evaluate('stage23d.data.regions.map(r=>r.name)');visited=[]
        for name in region_names:
            page.select_option('#region',name);visited.append(page.evaluate('stage23d.panels.every(p=>p.current.highlight.children.length===2)'))
        checks['seven_temporal_four_route_regions']=len(region_names)==11 and all(visited)
        page.select_option('#region','GROUP_CLUSTER_2');page.screenshot(path=str(out/'regional_overlay.png'),full_page=True)
        page.select_option('#estimate','batch');page.wait_for_function('stage23d.panels.every(p=>p.current===p.cache.get(p.label+"_batch"))');page.wait_for_timeout(600);checks['batch_real_trajectory_toggle']=page.evaluate('stage23d.metrics.loadedAssets===8 && stage23d.panels.every(p=>Object.values(p.current.state.trajectory).reduce((n,a)=>n+a.length,0)===5796)')
        page.click('#reset');page.select_option('#identity','all');page.click('#top');page.wait_for_timeout(600);page.screenshot(path=str(out/'batch_gauge_xy.png'),full_page=True)
        for _ in range(2):
            page.select_option('#estimate','incremental');page.wait_for_function('stage23d.panels.every(p=>p.current===p.cache.get(p.label+"_incremental"))');page.select_option('#estimate','batch');page.wait_for_function('stage23d.panels.every(p=>p.current===p.cache.get(p.label+"_batch"))')
        checks['buffers_cached_across_switches']=page.evaluate('stage23d.metrics.loadedAssets===8 && stage23d.metrics.bufferBuilds===8')
        page.uncheck('#maps');checks['map_visibility_toggle']=page.evaluate('stage23d.panels.every(p=>!p.current.maps.robot1.visible&&!p.current.maps.robot3.visible)');page.check('#maps')
        checks['explicit_final_only_not_live']=page.get_by_text('PROGRESSIVE_STATE_EXPORT_REQUIRED',exact=True).is_visible()
        before=page.evaluate('JSON.parse(JSON.stringify(stage23d.metrics))');page.evaluate('stage23d.dispose()');checks['resource_cleanup']=page.evaluate('stage23d.metrics.disposed && stage23d.panels.every(p=>p.cache.size===0&&p.scene.children.length===0)&&document.querySelectorAll("canvas").length===0')
        checks['no_browser_errors']=not errors and not failed
        report=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,errors=errors,failed_requests=failed,browser_version=browser.version,url=a.url,headless=True,renderer='software ANGLE SwiftShader; not foreground/GPU smoothness validation',presentation_metrics=before,total_browser_test_s=time.monotonic()-start,scientific_runtime_measured=False,screenshots={f.name:dict(bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in out.glob('*.png')})
        (out/'browser_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));browser.close()
        assert report['status']=='PASS',report
if __name__=='__main__':main()
