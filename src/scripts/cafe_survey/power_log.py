#!/usr/bin/env python3
"""Power logger for nevo's Pi 5: catches what happens in the seconds before a crash.

Every 0.5 s it appends one line to ~/power_logs/power_<date_time>.csv:
  time, 5V rail (EXT5V_V), throttle flags, CPU temp, load average, ARM clock
Each line is written with O_DSYNC, so it is on the SD card before the next
sample: after a power loss the file ends at (almost) the last moment.
No fsync of the whole disk, so it does not stall the bag recorder.

  python3 ~/cafe_survey/power_log.py          # Ctrl+C to stop
  python3 ~/cafe_survey/power_log.py --last   # after a crash: show the end of the newest log

Reading the result:
  sudden cut from a normal ~4.9 V      -> contact / connector problem (jolt, bump)
  slow sag over minutes, then the end  -> converter overheating or overloaded
  sharp dips when accelerating/climbing -> current spikes, converter too weak
  throttle bit 0x1 / 0x10000 set        -> the Pi saw under-voltage
"""
import argparse
import glob
import os
import subprocess
import time

LOGDIR = os.path.expanduser('~/power_logs')


def vc(*args):
    try:
        return subprocess.run(['vcgencmd', *args], capture_output=True, text=True,
                              timeout=2).stdout.strip()
    except Exception:
        return ''


def sample():
    v5 = vc('pmic_read_adc', 'EXT5V_V').split('=')[-1].rstrip('V')
    thr = vc('get_throttled').split('=')[-1]
    temp = vc('measure_temp').split('=')[-1].rstrip("'C")
    arm = vc('measure_clock', 'arm').split('=')[-1]
    load = open('/proc/loadavg').read().split()[0]
    return v5, thr, temp, load, arm


def show_last(n=40):
    logs = sorted(glob.glob(os.path.join(LOGDIR, 'power_*.csv')))
    if not logs:
        print('no logs in', LOGDIR)
        return
    path = logs[-1]
    lines = open(path).read().splitlines()
    print(f'{path}: {len(lines) - 1} samples')
    rows = [l.split(',') for l in lines[1:] if l.count(',') >= 5]
    if rows:
        v = [float(r[1]) for r in rows if r[1]]
        print(f'5V rail over the whole log: max {max(v):.3f}  min {min(v):.3f} V; '
              f'flags seen: {sorted(set(r[2] for r in rows))}')
    print(lines[0])
    print('\n'.join(lines[-n:]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--last', action='store_true', help='show the end of the newest log')
    ap.add_argument('--period', type=float, default=0.5)
    a = ap.parse_args()
    if a.last:
        show_last()
        return
    os.makedirs(LOGDIR, exist_ok=True)
    path = os.path.join(LOGDIR, time.strftime('power_%Y-%m-%d_%H%M%S.csv'))
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_DSYNC, 0o644)
    os.write(fd, b'time,ext5v_v,throttled,temp_c,load1,arm_hz\n')
    print(f'logging to {path} every {a.period} s, Ctrl+C to stop')
    vmin = 9.0
    try:
        while True:
            t0 = time.monotonic()
            v5, thr, temp, load, arm = sample()
            stamp = time.strftime('%H:%M:%S') + f'.{int(time.time() * 10) % 10}'
            os.write(fd, f'{stamp},{v5},{thr},{temp},{load},{arm}\n'.encode())
            try:
                vmin = min(vmin, float(v5))
            except ValueError:
                pass
            print(f'\r{stamp}  5V {v5} V (min {vmin:.3f})  flags {thr}  {temp} C  load {load}   ',
                  end='', flush=True)
            time.sleep(max(0.0, a.period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print(f'\nstopped; min 5V {vmin:.3f} V; log: {path}')
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
