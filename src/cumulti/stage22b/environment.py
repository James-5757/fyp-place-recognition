"""Post-run host/software metadata, not a runtime or physical-robot measurement."""
import importlib.metadata
import platform
import time
from common import *


def main():
    cpu=[line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')]
    versions={}
    for package in ['numpy','pandas','scipy','open3d','gtsam']:
        try:versions[package]=importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:versions[package]=None
    write_json(OUT/('environment_geometry.json' if versions['open3d'] else 'environment_backend.json'),
        dict(recorded_unix_ns=time.time_ns(),scope='post-run environment metadata; not per-call profiling',
             python=sys.version,executable=sys.executable,machine=platform.machine(),kernel=platform.release(),
             cpu_model=cpu[0] if cpu else None,logical_CPUs=os.cpu_count(),
             host_RAM_bytes=os.sysconf('SC_PHYS_PAGES')*os.sysconf('SC_PAGE_SIZE'),versions=versions,
             threads={k:os.environ.get(k) for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']},
             actual_physical_robot_CPU_memory='NOT_MEASURED'))

if __name__=='__main__':main()
