"""Read-only Linux pidfd timing observer; does not instrument registration."""
import argparse,select,time
from stage22a_common import *
def main(pid):
    fd=os.pidfd_open(pid) if hasattr(os,'pidfd_open') else None;clock_ticks=os.sysconf('SC_CLK_TCK')
    # PID comm may contain spaces; fields after ')' begin at stat field3.
    fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
    start_ticks=int(fields[19]);start_boot_seconds=start_ticks/clock_ticks
    dump('gicp_full_process_observer.json',dict(status='OBSERVING',pid=pid,kernel_start_ticks=start_ticks,clock_ticks_per_second=clock_ticks,scope='full Python verification process lifetime, includes imports/indexing/checkpoints; not algorithm critical path alone'))
    if fd is not None:
        poll=select.poll();poll.register(fd,select.POLLIN);poll.poll()
    else:
        # Some Python builds omit pidfd_open. Identity-check proc start ticks
        # to avoid confusing a reused PID with the original process.
        while True:
            try:
                current=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
                if int(current[19])!=start_ticks or current[0]=='Z':break
            except FileNotFoundError:break
            time.sleep(.25)
    elapsed=time.clock_gettime(time.CLOCK_BOOTTIME)-start_boot_seconds
    if fd is not None:os.close(fd)
    dump('gicp_full_process_observer.json',dict(status='PROCESS_EXIT_OBSERVED',pid=pid,measured_process_wall_seconds=elapsed,start_time_quantization_seconds=1/clock_ticks,exit_observation_resolution_seconds=0 if fd is not None else .25,includes_suspend=True,scope='kernel process start to exit observation; actual process-lifetime wall, NOT modelled projection',does_not_imply_success=True))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('pid',type=int);main(p.parse_args().pid)
