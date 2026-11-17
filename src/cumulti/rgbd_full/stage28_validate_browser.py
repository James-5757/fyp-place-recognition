"""Validate published Stage28 assets in Chromium and capture the actual maps."""
import argparse
import json
import time
from pathlib import Path

from playwright.sync_api import sync_playwright


def run(browser, base, dest, sensor, robot):
    errors=[]
    page=browser.new_page(viewport={'width': 1460,'height': 970})
    page.on('pageerror',lambda error: errors.append(str(error)))
    url=f'{base}?sensor={sensor}&robot={robot}&debug=1'
    page.goto(url,wait_until='domcontentloaded')
    page.wait_for_function("window.stage28 && !stage28.state().failed",timeout=30000)
    manifest=page.evaluate('stage28.manifest')
    prefix=f'{robot}_{sensor}'

    def state():
        return page.evaluate('stage28.state()')

    def seek(t):
        page.evaluate('(t)=>stage28.seek(t)',t)
        page.wait_for_function('(t)=>stage28.state().time===t && !stage28.state().applying',arg=t)
        return state()

    def image(name):
        page.evaluate('new Promise(ok=>requestAnimationFrame(()=>requestAnimationFrame(ok)))')
        page.screenshot(path=dest/(name+'.png'))

    checks=[]
    if robot!='combined':
        representative=manifest['boundaries'][len(manifest['boundaries'])//2]['epoch']
        for event in sorted(manifest['boundaries'],key=lambda e:e['t']):
            before=seek(event['t']-1e-7)
            after=seek(event['t'])
            chunks={file:(count,uid) for file,count,uid in after['chunks']}
            assert all(file in chunks and chunks[file][0]>=count and chunks[file][1]==uid
                       for file,count,uid in before['chunks']),event
            assert after['displayedPoints']>=before['displayedPoints'] and not event['large_gap'],event
            assert after['marks'][robot]['visible'],event
            checks.append(dict(epoch=event['epoch'],t=event['t'],before=before['displayedPoints'],
                               after=after['displayedPoints'],retained=True,source=event['source']))
            if event['epoch']==representative:
                page.locator('#orbit').click()
                image(prefix+'_boundary')
    end=seek(manifest['duration'])
    assert end['displayedPoints']==manifest['points'] and end['displayedChunks']==len(manifest['chunks'])
    assert set(end['marks'])==set(manifest['robots']) and all(r['visible'] for r in end['marks'].values())
    page.locator('#orbit').click()
    image(prefix+'_final')
    if sensor=='lidar' and robot!='combined':
        page.locator('#hybrid').check()
        page.locator('#top').click()
        image(prefix+'_bev')
        page.locator('#hybrid').uncheck()
    page.locator('#resetView').click()
    initial=state()
    page.mouse.move(700,350)
    page.mouse.down()
    page.mouse.move(780,390,steps=7)
    page.mouse.up()
    assert state()['camera']!=initial['camera']
    initial=state()
    page.mouse.wheel(0,-360)
    page.wait_for_function('(p)=>JSON.stringify(stage28.state().camera)!==JSON.stringify(p)',arg=initial['camera'])
    initial=state()
    page.mouse.down(button='right')
    page.mouse.move(825,440,steps=7)
    page.mouse.up(button='right')
    assert state()['target']!=initial['target']
    page.locator('#trajectories').uncheck()
    assert all(not r['visible'] for r in state()['marks'].values())
    page.locator('#trajectories').check()
    page.locator('#robots input').first.uncheck()
    assert not state()['marks'][manifest['robots'][0]]['visible']
    page.locator('#robots input').first.check()
    epoch=f"{manifest['robots'][0]}:{manifest['chunks'][0]['epoch']}"
    page.locator('#epoch').select_option(epoch)
    robot_prefix,epoch_id=epoch.split(':')
    assert all(file.startswith(f'{robot_prefix}-e{int(epoch_id):02d}-') for file,_,_ in state()['chunks'])
    page.locator('#epoch').select_option('')
    point_mid=seek(manifest['duration']/2)['displayedPoints']
    assert point_mid<end['displayedPoints']
    page.locator('#follow').select_option(manifest['robots'][0])
    page.locator('#follow').select_option('')
    page.locator('#reset').click()
    page.wait_for_function('stage28.state().time===0')
    seconds=None
    if robot!='combined':
        page.locator('#speed').select_option('4')
        tick=time.monotonic()
        page.locator('#play').click()
        page.wait_for_function('stage28.state().time===stage28.manifest.duration && !stage28.state().playing',
                               timeout=150000)
        seconds=time.monotonic()-tick
        assert state()['displayedPoints']==manifest['points']
    assert not errors,errors
    page.close()
    return dict(sensor=sensor,robot=robot,points=end['displayedPoints'],chunks=end['displayedChunks'],
                duration_s=manifest['duration'],epochs=len(manifest['transforms']),boundaries=checks,
                uninterrupted_4x_wall_s=seconds,actual_3d_interactions_pass=True,rewind_pass=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('url')
    parser.add_argument('destination',type=Path)
    parser.add_argument('--channel',default='msedge')
    parser.add_argument('--sensor',choices=('lidar','rgbd'))
    args=parser.parse_args()
    args.destination.mkdir(parents=True,exist_ok=True)
    with sync_playwright() as p:
        browser=p.chromium.launch(channel=args.channel,args=['--enable-unsafe-swiftshader','--use-angle=swiftshader'])
        for sensor in (args.sensor,) if args.sensor else ('lidar','rgbd'):
            reports=[]
            for view in ('robot1','robot3','combined'):
                report=run(browser,args.url,args.destination,sensor,view)
                print(sensor,view,'PASS',flush=True)
                reports.append(report)
            (args.destination/f'{sensor}_browser_validation.json').write_text(json.dumps(
                dict(status='PASS',browser=browser.version,views=reports),indent=2)+'\n',encoding='utf-8')
        browser.close()
