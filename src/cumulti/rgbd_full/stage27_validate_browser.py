"""Validate Stage27 in the actual WebGL replay."""
import argparse
import json
import time
from pathlib import Path

from playwright.sync_api import sync_playwright


def validate(url, dest, browser):
    dest.mkdir(parents=True, exist_ok=True)
    errors = []
    page = browser.new_page(viewport={'width': 1460, 'height': 940})
    page.on('pageerror', lambda error: errors.append(str(error)))
    page.goto(url, wait_until='domcontentloaded')
    page.wait_for_function("window.stage27 && !stage27.state().failed", timeout=30000)
    manifest = page.evaluate("stage27.manifest")

    def seek(t):
        page.evaluate("(t)=>stage27.seek(t)", t)
        page.wait_for_function("(t)=>!stage27.state().applying && stage27.state().time===t", arg=t)
        return page.evaluate("stage27.state()")

    def screenshot(name, mode):
        page.locator('#'+mode).click()
        page.evaluate("new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)))")
        page.screenshot(path=dest/(name+'.png'))

    seek(20)
    screenshot('begin', 'orbit')
    checks = []
    for event in sorted(manifest['boundaries'], key=lambda item: item['t']):
        before = seek(event['t']-1e-7)
        after = seek(event['t'])
        retained = {name:(count,uid) for name,count,uid in after['chunks']}
        assert all(name in retained and retained[name][0]>=count and retained[name][1]==uid
                   for name,count,uid in before['chunks']), event
        assert after['displayedPoints']>=before['displayedPoints'], event
        marker = after['marks'][event['robot']]
        assert marker['visible'] and any(abs(v)>1e-6 for v in marker['xyz']) and not event['large_gap'], event
        checks.append(dict(robot=event['robot'], epoch=event['epoch'], t=event['t'], source=event['source'],
                           points_before=before['displayedPoints'], points_after=after['displayedPoints'],
                           previous_chunks_preserved=True))
        if (event['robot'],event['epoch']) in {('robot1',1),('robot3',11),('robot3',19)}:
            screenshot(f"{event['robot']}_epoch_{event['epoch']}", 'top')
    mid = seek(manifest['duration']/2)
    screenshot('middle', 'orbit')
    assert all(m['visible'] for m in mid['marks'].values())
    page.locator('#end').click()
    page.wait_for_function("stage27.state().time===stage27.manifest.duration && !stage27.state().applying")
    final = page.evaluate('stage27.state()')
    assert final['displayedPoints']==manifest['points'] and final['displayedChunks']==len(manifest['chunks'])
    assert all(m['visible'] for m in final['marks'].values())
    screenshot('end', 'top')
    page.locator('#density').select_option('reduced')
    assert page.evaluate('stage27.state().displayedPoints')==manifest['points']
    page.locator('#density').select_option('full')
    page.locator('#robots input').nth(0).uncheck()
    assert not page.evaluate('stage27.state().marks.robot1.visible')
    page.locator('#robots input').nth(0).check()
    page.locator('#epoch').select_option('robot3:19')
    assert page.evaluate("stage27.state().chunks.length && stage27.state().chunks.every(([file])=>file.startsWith('robot3-e19-'))")
    page.locator('#epoch').select_option('')
    rewind = seek(manifest['duration']/2)
    assert rewind['displayedPoints']==mid['displayedPoints']<final['displayedPoints']
    page.locator('#speed').select_option('4')
    page.locator('#play').click()
    page.wait_for_function("(t)=>stage27.state().time>t+1",arg=rewind['time'])
    page.locator('#play').click()
    assert not page.evaluate('stage27.state().playing')
    page.locator('#reset').click()
    page.wait_for_function("stage27.state().time===0")
    started = time.monotonic()
    page.locator('#play').click()
    page.wait_for_function("stage27.state().time===stage27.manifest.duration && !stage27.state().playing",
                           timeout=150000)
    uninterrupted_seconds = time.monotonic() - started
    assert page.evaluate('stage27.state().displayedPoints===stage27.manifest.points')
    assert not errors, errors
    report = dict(status='PASS', browser=browser.version, boundaries=checks, total_epochs=len(manifest['transforms']),
                  final_time_s=final['time'], total_chunks=final['displayedChunks'], total_points=final['displayedPoints'],
                  final_visible_robots=list(final['marks']), seek_rewind_pass=True, playback_controls_pass=True,
                  layer_controls_pass=True, uninterrupted_4x_wall_s=uninterrupted_seconds, screenshots=sorted(p.name for p in dest.glob('*.png')))
    (dest/'browser_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    page.close()


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('url', help='Continuous replay URL, with ?debug=1')
    parser.add_argument('destination', type=Path)
    parser.add_argument('--channel', default='msedge')
    args = parser.parse_args()
    with sync_playwright() as p:
        browser = p.chromium.launch(channel=args.channel,args=['--enable-unsafe-swiftshader','--use-angle=swiftshader'])
        validate(args.url,args.destination,browser)
        browser.close()
