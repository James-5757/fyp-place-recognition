"""Actual HTTP/WebGL acceptance, separate from full tracking qualification."""
import argparse,json,time
from pathlib import Path
from playwright.sync_api import sync_playwright

def run(url,out):
 out=Path(out);out.mkdir(parents=True,exist_ok=True);errors=[];checks={};samples=[]
 with sync_playwright() as p:
  browser=p.chromium.launch(headless=True,args=['--use-angle=swiftshader','--enable-webgl'])
  page=browser.new_page(viewport={'width':1600,'height':1100},device_scale_factor=1)
  page.on('pageerror',lambda e:errors.append(str(e)))
  begin=time.monotonic();page.goto(url,wait_until='networkidle',timeout=120000);page.wait_for_function('window.rgbdFull!==undefined',timeout=60000)
  loading=time.monotonic()-begin;initial=page.evaluate('rgbdFull.snapshot()')
  checks['LAZY_RGB_STARTUP']=initial['stats']['rgbRequests']<=6
  metadata=page.evaluate('async()=>await Promise.all(Object.values((await (await fetch("manifest.json")).json()).robots).map(async p=>await(await fetch(p)).json()))')
  duration=max(d['duration_s'] for d in metadata);times={0.,duration,1972.508,duration/2}
  for d in metadata:
   times.add(d['duration_s']);previous=-1
   for f in d['frames']:
    if f['valid'] and f['epoch']!=previous:
     times.add(f['t']);times.add(min(duration,f['t']+2));previous=f['epoch']
  times=sorted(t for t in times if 0<=t<=duration)
  # Exercise every real epoch transition, then old-state seeks to trigger eviction.
  for t in times+list(reversed(times[::max(1,len(times)//12)])):
   started=time.monotonic();page.evaluate('(t)=>rgbdFull.setTime(t)',t)
   expected_epochs=[]
   for d in metadata:
    valid=[f for f in d['frames'] if f['t']<=min(t,d['duration_s']) and f['valid']];expected_epochs.append(valid[-1]['epoch'] if valid else None)
   page.wait_for_function('(expected)=>rgbdFull.snapshot().panels.every((p,i)=>expected[i]===null?!p.visiblePoints:p.epoch===expected[i])',arg=expected_epochs,timeout=15000)
   page.wait_for_timeout(100)
   snap=page.evaluate('rgbdFull.snapshot()');snap['seek_response_s']=time.monotonic()-started;samples.append(snap)
   for panel,d in zip(snap['panels'],metadata):
    expected=max(i for i,f in enumerate(d['frames']) if f['t']<=min(t,d['duration_s']))
    assert panel['index']==expected,('sensor index mismatch',t,panel,expected)
    assert panel['maxVisibleFirst'] is None or panel['maxVisibleFirst']<=expected,('future map point',t,panel)
    assert panel['maxPathEnd'] is None or panel['maxPathEnd']<=expected,('future trajectory',t,panel)
    if panel['rgbPreviewIndex']>=0:assert d['previews'][panel['rgbPreviewIndex']]['t']<=min(t,d['duration_s'])
    assert panel['cache']<=2 and panel['imageCache']<=12
  checks['SEEK_NATIVE_INDEX_AND_NO_FUTURE_POINTS']=True;checks['CACHE_BOUNDS']=True
  page.evaluate('(t)=>rgbdFull.setTime(t)',duration);page.wait_for_timeout(1200)
  for i,selector in enumerate(['#robot1 .map canvas','#robot3 .map canvas']):
   canvas=page.locator(selector);box=canvas.bounding_box();x=box['x']+box['width']*.5;y=box['y']+box['height']*.5
   before=page.evaluate('rgbdFull.snapshot()')['panels'][i];page.mouse.move(x,y);page.mouse.down();page.mouse.move(x+90,y+30,steps=8);page.mouse.up();page.wait_for_timeout(400);orbit=page.evaluate('rgbdFull.snapshot()')['panels'][i]
   assert before['camera']!=orbit['camera'],'Orbit does not move actual camera'
   page.mouse.wheel(0,-250);page.wait_for_timeout(400);zoom=page.evaluate('rgbdFull.snapshot()')['panels'][i];assert orbit['camera']!=zoom['camera']
   page.mouse.move(x,y);page.mouse.down(button='right');page.mouse.move(x+45,y+20,steps=6);page.mouse.up(button='right');page.wait_for_timeout(400);pan=page.evaluate('rgbdFull.snapshot()')['panels'][i];assert zoom['target']!=pan['target']
  checks['BOTH_PANELS_ORBIT_ZOOM_PAN']=True
  page.screenshot(path=str(out/'full_recordings_final.png'),full_page=True)
  for speed in ['0.5','1','2','4']:
   page.locator('#reset').click();page.locator('#speed').select_option(speed);page.locator('#play').click();page.wait_for_timeout(1200);page.locator('#play').click();t=page.evaluate('rgbdFull.snapshot().time');assert t>float(speed)*3,('Playback clock failed',speed,t)
  checks['PLAY_PAUSE_RESET_ALL_SPEEDS']=True
  page.evaluate('(t)=>rgbdFull.setTime(t)',duration/2);page.wait_for_timeout(1000);page.screenshot(path=str(out/'full_recordings_midpoint.png'),full_page=True)
  if len(times)>2:page.evaluate('(t)=>rgbdFull.setTime(t)',times[2]);page.wait_for_timeout(800)
  page.screenshot(path=str(out/'map_epoch_tracking_state.png'),full_page=True)
  page.evaluate('(t)=>rgbdFull.setTime(t)',duration);page.wait_for_timeout(800);final=page.evaluate('rgbdFull.snapshot()')
  checks['SHORTER_STREAM_END_EXPLICIT']=all('RECORDING COMPLETE' in p['state'] for p,d in zip(final['panels'],metadata) if d['duration_s']<duration)
  assert not final['stats']['errors'] and not errors,(errors,final['stats']['errors'])
  checks['NO_BROWSER_ERRORS']=True;checks['LOCAL_MAPS_PERSISTENT']=page.locator('article .map canvas').count()==2
  start=page.evaluate('rgbdFull.snapshot().renderFrames');page.wait_for_timeout(3000);fps=(page.evaluate('rgbdFull.snapshot().renderFrames')-start)/3
  result=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,loading_s=loading,measured_idle_render_fps=fps,software_WebGL=True,seek_cases=len(samples),seek_response_s=[s['seek_response_s'] for s in samples],final=final,errors=errors,tracking_quality='NOT inferred from rendering PASS')
  (out/'browser_validation.json').write_text(json.dumps(result,indent=2)+'\n');browser.close()
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('url');p.add_argument('out');a=p.parse_args();print(json.dumps(run(a.url,a.out),indent=2))
