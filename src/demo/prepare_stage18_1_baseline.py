#!/usr/bin/env python3
"""Reconstruct accepted Stage18 browser benchmark without changing its renderer."""
import subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
BASE='0447a62210d46a685285b8f6d649d49fd880960d'
def main():
    target=ROOT/'demo/progressive_multi_robot/benchmark_before'
    target.mkdir(exist_ok=True)
    for name in ['app.js','styles.css','index.html']:
        data=subprocess.check_output(['git','show',f'{BASE}:demo/progressive_multi_robot/{name}'],cwd=ROOT)
        if name=='index.html':data=data.replace(b'<head>',b'<head><base href="../">').replace(b'<script',b'<script defer src="benchmark_before/profiler.js"></script><script',1).replace(b'src="app.js"',b'src="benchmark_before/app.js"').replace(b'href="styles.css"',b'href="benchmark_before/styles.css"')
        (target/name).write_bytes(data)
    (target/'profiler.js').write_bytes((ROOT/'src/demo/measure_stage18_baseline.js').read_bytes())
    print('Serve parent demo root; open /benchmark_before/; only instrumentation differs.')
if __name__=='__main__':main()
