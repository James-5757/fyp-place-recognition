"""Download only the pinned public semantic checkpoint; never modify SLAM envs."""
import argparse,hashlib,json,urllib.request
from pathlib import Path
REPO='nvidia/segformer-b0-finetuned-ade-512-512'
REV='489d5cd81a0b59fab9b7ea758d3548ebe99677da'
EXPECTED_WEIGHTS_SHA='6ae39addd01de6b1b8bde2cf677d43a5cd733424b8d186de3f95d1c51fee23f9'

def main():
    p=argparse.ArgumentParser();p.add_argument('--destination',type=Path,default=Path.home()/'.cache/stage24_model');args=p.parse_args();args.destination.mkdir(parents=True,exist_ok=True)
    for n in ['config.json','preprocessor_config.json','model.safetensors']:
        dst=args.destination/n
        if not dst.exists():urllib.request.urlretrieve(f'https://huggingface.co/{REPO}/resolve/{REV}/{n}',dst)
        if n=='model.safetensors' and hashlib.sha256(dst.read_bytes()).hexdigest()!=EXPECTED_WEIGHTS_SHA:
            raise RuntimeError('Pinned checkpoint SHA256 mismatch')
        print(n,hashlib.sha256(dst.read_bytes()).hexdigest())
    (args.destination/'revision.json').write_text(json.dumps(dict(repo=REPO,revision=REV)))

if __name__=='__main__':main()
