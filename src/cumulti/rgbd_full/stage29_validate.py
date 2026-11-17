"""Verify same scale/origin and capture actual WebGL views without rebuilding maps."""
import argparse
import json
from math import isclose
from pathlib import Path

from playwright.sync_api import sync_playwright


def run(browser, url, dest):
    page=browser.new_page(viewport={'width':1460,'height':980})
    errors=[]
    page.on('pageerror',lambda error:errors.append(str(error)))
    records={}

    def open_view(robot):
        page.goto(f'{url}?sensor=rgbd&robot={robot}&debug=1',wait_until='domcontentloaded')
        page.wait_for_function('window.stage29 && !stage29.state().failed')
        return page.evaluate('stage29.manifest')

    def timeline(t):
        page.evaluate('(t)=>stage29.seek(t)',t)
        page.wait_for_function('(t)=>stage29.state().time===t && !stage29.state().applying',arg=t)
        return page.evaluate('stage29.state()')

    def shot(name):
        page.evaluate('new Promise(ok=>requestAnimationFrame(()=>requestAnimationFrame(ok)))')
        page.screenshot(path=dest/(name+'.png'))

    ordered=[]
    for robot in ('robot1','robot3','combined'):
        manifest=open_view(robot)
        end=timeline(manifest['duration'])
        assert end['displayedPoints']==manifest['points']
        page.locator('#top').click()
        shot(robot+'_auto')
        page.locator('#cameraMode').select_option('metric')
        page.locator('#boxes').check()
        page.locator('#top').click()
        state=page.evaluate('stage29.state()')
        assert end['displayedPoints']==state['displayedPoints']
        shot(robot+'_metric')
        ordered.append([state['span'],*state['projection'],*state['camera'],*state['target']])
        records[robot]=dict(points=end['displayedPoints'],chunks=end['displayedChunks'],vertical_span_m=state['span'])
    assert all(isclose(a,b,rel_tol=1e-12,abs_tol=1e-12) for row in ordered[1:] for a,b in zip(ordered[0],row)), ordered
    page.locator('#robots input').first.uncheck()
    assert not page.evaluate('stage29.state().marks.robot1.visible')
    page.locator('#robots input').first.check()
    timeline(0)
    page.locator('#play').click()
    page.wait_for_function('stage29.state().playing && stage29.state().time>1')
    page.locator('#play').click()
    assert not page.evaluate('stage29.state().playing')
    page.locator('#resetView').click()
    before=page.evaluate('stage29.state()')
    page.mouse.move(600,350)
    page.mouse.down()
    page.mouse.move(690,390,steps=5)
    page.mouse.up()
    assert page.evaluate('stage29.state().camera')!=before['camera']
    before=page.evaluate('stage29.state()')
    page.mouse.wheel(0,-320)
    page.wait_for_function('(span)=>stage29.state().span!==span',arg=before['span'])
    before=page.evaluate('stage29.state()')
    page.mouse.down(button='right')
    page.mouse.move(725,408,steps=5)
    page.mouse.up(button='right')
    assert page.evaluate('stage29.state().target')!=before['target']

    manifest=open_view('robot3')
    end=timeline(manifest['duration'])
    page.locator('#localEpoch').check()
    local=[]
    for row in manifest['transforms']:
        epoch=row['epoch']
        page.locator('#epoch').select_option(f'robot3:{epoch}')
        selected=page.evaluate('stage29.state()')
        assert all(f'robot3-e{epoch:02d}-' in r[0] for r in selected['chunks'])
        assert selected['displayedPoints']>0
        shot(f'robot3_local_e{epoch:02d}')
        local.append(dict(epoch=epoch,points=selected['displayedPoints']))
    page.locator('#localEpoch').uncheck()
    page.locator('#epoch').select_option('')
    before=timeline(manifest['duration']/2)
    after=timeline(manifest['duration'])
    assert before['displayedPoints']<after['displayedPoints']==end['displayedPoints']
    for event in manifest['boundaries']:
        before=timeline(event['t']-1e-7)
        after=timeline(event['t'])
        active={file:(count,uid) for file,count,uid in after['chunks']}
        assert all(file in active and active[file][0]>=count and active[file][1]==uid
                   for file,count,uid in before['chunks'])
    assert not errors, errors
    (dest/'browser_validation.json').write_text(json.dumps(dict(
        status='PASS',browser=browser.version,views=records,common_projection=True,common_camera_origin=True,
        metric_orbit_zoom_pan=True,robot3_local_epochs=local,robot3_boundaries_checked=len(manifest['boundaries']),
        final_chunk_counts_unchanged=True),indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('url')
    p.add_argument('destination',type=Path)
    a=p.parse_args()
    a.destination.mkdir(parents=True,exist_ok=True)
    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(channel='msedge',args=['--enable-unsafe-swiftshader','--use-angle=swiftshader'])
        try:
            run(browser,a.url,a.destination)
        finally:
            browser.close()
