"""Independent browser test. Runs on the workstation with installed Playwright."""
import argparse,json,time,math
from pathlib import Path
from playwright.sync_api import sync_playwright

def main():
    p=argparse.ArgumentParser();p.add_argument('--base-url',default='http://127.0.0.1:8021/stage24_rgb_semantic/');p.add_argument('--out-dir',type=Path,required=True);p.add_argument('--chromium',required=True);args=p.parse_args();args.out_dir.mkdir(parents=True,exist_ok=True)
    errors=[];failures=[];checks=[]
    with sync_playwright() as pw:
        browser=pw.chromium.launch(executable_path=args.chromium,headless=True,args=['--use-angle=swiftshader','--enable-webgl'])
        page=browser.new_page(viewport={'width':1500,'height':1120});page.on('pageerror',lambda e:errors.append(str(e)));page.on('requestfailed',lambda r:failures.append(dict(url=r.url,error=r.failure)))
        start=time.monotonic();page.goto(args.base_url);page.wait_for_function('window.stage24Audit?.().ready',timeout=60000);startup=(time.monotonic()-start)*1000
        page.wait_for_function('document.getElementById("rgb").complete&&document.getElementById("semanticImage").complete&&document.getElementById("rgb").naturalWidth>0')
        def audit():return page.evaluate('window.stage24Audit()')
        for mode in ['robot','rgb','semantic']:
            page.locator('#mode').select_option(mode);a=audit();assert a['mode']==mode and a['geometry_builds']==2
            if mode!='robot':assert a['observed_RGB_3D']==a['labeled_semantic_3D']==0 and a['unknown_fraction']==1
            page.screenshot(path=str(args.out_dir/f'{mode}_map.png'),full_page=True);checks.append(dict(mode=mode,PASS=True))
        page.locator('#r3').uncheck();assert audit()['displayed_points']['robot3']==0;page.locator('#r3').check()
        page.locator('#sample').select_option('0');page.wait_for_function('document.getElementById("rgb").complete&&document.getElementById("rgb").naturalWidth>0')
        page.screenshot(path=str(args.out_dir/'opening_overexposure.png'),full_page=True)
        page.locator('#sample').select_option('8');page.wait_for_function('document.getElementById("rgb").complete&&document.getElementById("rgb").naturalWidth>0')
        page.get_by_role('button',name='Perspective',exact=True).click();page.wait_for_timeout(1500)
        canvas=page.locator('#canvas').bounding_box();x=canvas['x']+canvas['width']*.55;y=canvas['y']+canvas['height']*.60
        before=audit();page.mouse.move(x,y);page.mouse.down();page.mouse.move(x+100,y+45,steps=12);page.mouse.up();page.wait_for_timeout(1200);rot=audit()
        assert math.dist(before['camera_position'],rot['camera_position'])>1
        def distance(a):return math.dist(a['camera_position'],a['camera_target'])
        page.mouse.move(x,y);page.mouse.wheel(0,-200);page.wait_for_timeout(1200);zoom=audit();assert distance(zoom)<distance(rot)*.99
        page.mouse.move(x,y);page.mouse.down(button='right');page.mouse.move(x+65,y+25,steps=10);page.mouse.up(button='right');page.wait_for_timeout(1200);pan=audit();assert math.dist(zoom['camera_target'],pan['camera_target'])>.1
        a=audit();page.wait_for_timeout(1200);b=audit();fps=(b['render_frames']-a['render_frames'])/1.2
        assert not errors and not failures,(errors,failures)
        result=dict(status='PASS',scope='presentation functional test only; real RGB/semantic3D mapping remains BLOCKED',checks=checks,
            control_checks=dict(orbit=True,zoom=True,pan=True,source_toggle=True,image_sample_switch=True),startup_ms=startup,
            measured_headless_render_FPS=fps,render_backend='headless Chromium SwiftShader software WebGL; not foreground hardware smoothness proof',
            geometry_builds=b['geometry_builds'],page_errors=errors,request_failures=failures,scientific_geometry_changed=False,
            screenshots=[f'{m}_map.png' for m in ['robot','rgb','semantic']])
        (args.out_dir.parent/'demo_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True);browser.close()

if __name__=='__main__':main()
