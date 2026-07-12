#!/usr/bin/env python3
"""
collect_pid_step_response.py
=============================================================
BAB IV - TEST 6: PID Motor Control Response (FORMAL DATA).

Difference vs characterization/pid_step_response.py:
    That script is for TUNING (gives suggestions, one speed/run).
    THIS script collects the formal Chapter IV data:
      - sweeps the spec speeds 0.1, 0.15, 0.2 m/s automatically
      - repeats each speed N times
      - computes steady-state RMSE = sqrt(mean((v_target - v_actual)^2))
        in addition to rise time, overshoot, steady-state error
      - records the Kp/Ki/Kd you pass in (STM32 does not report them)
      - saves a tidy CSV + a per-speed step-response plot
      - auto-computes mean/std/min/max across repetitions

PROCEDURE:
    1. Wheels OFF the ground (bench test) OR clear straight space.
    2. Kill ROS first (talks to STM32 serial directly):
         ~/Autonomous-Mobile-Robot/ws_amr_robot/src/robot_bringup/scripts/tools/kill_robot.sh
    3. Run:
         python3 collect_pid_step_response.py --reps 5 --kp 2.0 --ki 1.5 --kd 0.02

ARGS:
    --reps      repetitions per speed (default 5)
    --speeds    target speeds in m/s (default 0.1 0.15 0.2)
    --duration  seconds of recording per step (default 4.0)
    --kp --ki --kd   current firmware gains, recorded into the CSV
    --label     scenario label for the output filename
=============================================================
"""

import os
import sys
import time
import json
import math
import argparse

import serial

# shared helpers (constants, RMSE, stats, CSV, timestamped paths)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import amr_test_utils as U

# ============================================================
# CONFIGURATION
# ============================================================
SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD_RATE   = 115200

MM_PER_TICK = U.M_PER_TICK * 1000.0   # reuse the single source of truth

SETTLE_BEFORE = 0.5   # s at zero before step
SETTLE_AFTER  = 1.0   # s coast down after step
MAX_PHYSICAL_SPEED = 300.0  # mm/s; readings beyond this are telemetry glitches


# ============================================================
# SERIAL HELPERS  (same protocol as pid_step_response.py)
# ============================================================
def connect_serial():
    print(f"Connecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.3)
    print("  Connected.")
    return ser


def send_velocity(ser, v_mmps, w_mradps=0):
    cmd = f"V:{int(v_mmps)},W:{int(w_mradps)}\r\n"
    ser.write(cmd.encode('utf-8'))


def set_gains_live(ser, kp, ki, kd, timeout_s=1.5):
    """Send the K: live-gain command (integers x100) and wait for the
    firmware [GAINS:...] echo. Returns True if confirmed."""
    kp100, ki100, kd100 = int(round(kp * 100)), int(round(ki * 100)), int(round(kd * 100))
    ser.reset_input_buffer()
    ser.write(f"K:{kp100},{ki100},{kd100}\r\n".encode('utf-8'))
    expected = f"[GAINS:{kp100},{ki100},{kd100}]"
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except UnicodeDecodeError:
            continue
        if line == expected:
            print(f"  Gains confirmed by firmware: {expected}")
            return True
    print(f"  WARNING: no [GAINS] echo (expected {expected}). "
          f"Old firmware without K: support? Gains NOT changed.")
    return False


def drain_and_get_latest(ser):
    latest = None
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('{') and line.endswith('}'):
                latest = json.loads(line)
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
    return latest


def read_all_packets(ser):
    """Return EVERY complete JSON telemetry packet waiting in the buffer,
    in order (not just the latest). Reading every 20Hz frame and using the
    firmware's own 'dt' (ms this frame covers) removes the 20Hz-vs-poll-rate
    aliasing that produced the 78/153 sawtooth."""
    pkts = []
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('{') and line.endswith('}'):
                pkts.append(json.loads(line))
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
    return pkts


def tick_delta(curr, prev):
    """int16 encoder counter difference with wraparound handling."""
    d = curr - prev
    if d >  32767: d -= 65536
    if d < -32768: d += 65536
    return d


CORRUPT_TICKS = 5000  # |ticks/frame| above this = truly corrupt counter
                      # (~1.5m in one frame, physically impossible). A merely
                      # DELAYED frame has a big-but-real delta and must be kept,
                      # else its ticks are lost and speed under-reports (the bug
                      # the ground-truth tape test exposed, 2026-06-21).


class SpeedTracker:
    """Smooth, ground-truth-validated wheel speed from raw telemetry.

    Accumulates wrap-corrected ticks into a running total (the absolute counter
    makes this self-correcting across dropped/delayed frames), then reports
    speed as (ticks moved over the last `window` seconds) / (that real
    wall-clock span). Validated against tape+clock: matches within ~2%.
    """
    def __init__(self, window_s):
        from collections import deque
        self.window = window_s
        self.cum_l = 0
        self.cum_r = 0
        self.prev = None
        self.buf = deque()   # (t, cum_l, cum_r)

    def update(self, packet, t):
        """Feed one telemetry packet at wall-time t (s). Returns (l_mmps,
        r_mmps) once enough span has accumulated, else None."""
        if self.prev is None:
            self.prev = packet
            self.buf.append((t, 0, 0))
            return None
        dl =  tick_delta(packet['l'], self.prev['l'])
        dr = -tick_delta(packet['r'], self.prev['r'])   # right polarity = -1
        self.prev = packet
        if abs(dl) > CORRUPT_TICKS or abs(dr) > CORRUPT_TICKS:
            return None      # corrupt counter only; do NOT touch cum/prev-time
        # ALWAYS accumulate real motion (delayed frames included)
        self.cum_l += dl
        self.cum_r += dr
        self.buf.append((t, self.cum_l, self.cum_r))
        # keep just enough history to span `window`
        while len(self.buf) >= 2 and (t - self.buf[1][0]) >= self.window:
            self.buf.popleft()
        t0, l0, r0 = self.buf[0]
        span = t - t0
        if span < 0.04:
            return None
        l_spd = (self.cum_l - l0) * MM_PER_TICK / span
        r_spd = (self.cum_r - r0) * MM_PER_TICK / span
        # reject only the emitted SAMPLE if absurd; cumulative stays intact
        if abs(l_spd) > MAX_PHYSICAL_SPEED or abs(r_spd) > MAX_PHYSICAL_SPEED:
            return None
        return (l_spd, r_spd)


# ============================================================
# ONE STEP RECORDING
# ============================================================
def record_step(ser, target_mmps, duration_s):
    """Record one forward step. Returns list of sample dicts:
    time_s, target_mmps, left_speed_mmps, right_speed_mmps."""
    samples = []

    # settle at zero
    t0 = time.time()
    while (time.time() - t0) < SETTLE_BEFORE:
        send_velocity(ser, 0)
        time.sleep(0.05)
    ser.reset_input_buffer()

    # baseline packet
    first = None
    deadline = time.time() + 1.0
    while time.time() < deadline:
        send_velocity(ser, 0)
        d = drain_and_get_latest(ser)
        if d is not None and 'l' in d and 'r' in d:
            first = d
            break
        time.sleep(0.05)
    if first is None:
        print("  ERROR: no telemetry from STM32.")
        return []

    # step phase: every frame -> sliding 0.15s wall-clock window for clean speed
    t_start = time.time()
    tracker = SpeedTracker(window_s=0.15)
    tracker.update(first, 0.0)
    while (time.time() - t_start) < duration_s:
        send_velocity(ser, target_mmps)
        time.sleep(0.02)
        for data in read_all_packets(ser):
            if 'l' not in data or 'r' not in data:
                continue
            now = time.time() - t_start
            spd = tracker.update(data, now)
            if spd is None:
                continue
            l_speed, r_speed = spd
            samples.append({
                'time_s':           round(now, 4),
                'target_mmps':      target_mmps,
                'left_speed_mmps':  round(l_speed, 2),
                'right_speed_mmps': round(r_speed, 2),
            })

    # coast down (motors off)
    t_c = time.time()
    while (time.time() - t_c) < SETTLE_AFTER:
        send_velocity(ser, 0)
        time.sleep(0.05)

    return samples


# ============================================================
# METRICS (per wheel, per step)  -- adds RMSE
# ============================================================
def analyze(samples, target_mmps, wheel):
    key = f'{wheel}_speed_mmps'
    active = [s for s in samples if s['target_mmps'] == target_mmps]
    if len(active) < 5:
        return None

    speeds = [s[key] for s in active]
    times  = [s['time_s'] for s in active]

    # moving-average smoothing for rise/overshoot (RMSE uses raw data)
    w = 5
    sm = []
    for i in range(len(speeds)):
        a = max(0, i - w // 2)
        b = min(len(speeds), i + w // 2 + 1)
        sm.append(sum(speeds[a:b]) / (b - a))

    # steady-state window = last 30% of active samples (compute FIRST: rise and
    # overshoot are conventionally referenced to the ACHIEVED final value, not
    # the setpoint -- using the setpoint breaks rise time when there is any
    # steady-state error, e.g. ss=89 never crosses 90% of target=100).
    ss_i = int(len(active) * 0.7)
    ss_speeds = speeds[ss_i:]            # RAW, for RMSE
    ss_mean = sum(ss_speeds) / max(1, len(ss_speeds))
    ss_err = abs(target_mmps) - abs(ss_mean)
    ss_err_pct = ss_err / abs(target_mmps) * 100

    # rise time: first reach 90% of the ACHIEVED steady-state speed
    thr = abs(ss_mean) * 0.9
    rise = None
    for t, s in zip(times, sm):
        if abs(s) >= thr:
            rise = t
            break

    # overshoot relative to achieved steady-state (0 if it never exceeds it).
    # Search for the peak only within a TIME-bounded transient window (2x rise
    # time, or a 3s fallback if rise wasn't found), NOT a percentage-of-samples
    # window like ss_i. A percentage window scales with total run duration, so
    # on a long run (e.g. 20s) it can span 10+ seconds of already-settled data
    # -- long enough for an unrelated delayed/noisy telemetry frame to get
    # picked up as "peak", inflating overshoot the longer the recording runs.
    # A physical transient only lasts a few multiples of the rise time.
    transient_end = (rise * 2.0) if rise is not None else 3.0
    transient = [s for t, s in zip(times, sm) if t <= transient_end] or sm
    peak = max(transient) if target_mmps > 0 else min(transient)
    overshoot = max(0.0, (abs(peak) - abs(ss_mean)) / abs(ss_mean) * 100) \
        if ss_mean else 0.0

    # RMSE during steady state: sqrt(mean((v_target - v_actual)^2))
    ss_rmse = U.rmse([abs(target_mmps) - abs(v) for v in ss_speeds])

    return {
        'wheel':          wheel,
        'rise_time_s':    round(rise, 3) if rise is not None else None,
        'overshoot_pct':  round(overshoot, 1),
        'ss_speed_mmps':  round(ss_mean, 1),
        'ss_error_mmps':  round(ss_err, 1),
        'ss_error_pct':   round(ss_err_pct, 1),
        'ss_rmse_mmps':   round(ss_rmse, 2),
    }


# ============================================================
# PLOT (one figure per speed, last repetition)
# ============================================================
def plot_step(samples, target_mmps, png_path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  matplotlib not installed, skipping plot "
              "(pip3 install matplotlib --break-system-packages)")
        return
    if not samples:
        return
    t = [s['time_s'] for s in samples]
    plt.figure(figsize=(11, 5))
    plt.plot(t, [s['left_speed_mmps'] for s in samples],
             color='#2196F3', lw=1.5, label='Left actual')
    plt.plot(t, [s['right_speed_mmps'] for s in samples],
             color='#F44336', lw=1.5, label='Right actual')
    plt.axhline(target_mmps, color='#4CAF50', ls='--', lw=2, label='Target')
    tol = abs(target_mmps) * 0.1
    plt.axhspan(target_mmps - tol, target_mmps + tol, color='green', alpha=0.1)
    plt.title(f'PID Step Response  |  Target {target_mmps} mm/s '
              f'({target_mmps/1000:.2f} m/s)')
    plt.xlabel('Time (s)'); plt.ylabel('Speed (mm/s)')
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig(png_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Plot: {png_path}")


# ============================================================
# MAIN
# ============================================================
def main():
    p = argparse.ArgumentParser(description='BAB IV Test 6: PID step response data')
    p.add_argument('--reps', type=int, default=5)
    p.add_argument('--speeds', type=float, nargs='+', default=[0.1, 0.15, 0.2],
                   help='target speeds in m/s')
    p.add_argument('--duration', type=float, default=4.0)
    p.add_argument('--kp', type=float, required=True, help='current firmware Kp')
    p.add_argument('--ki', type=float, required=True, help='current firmware Ki')
    p.add_argument('--kd', type=float, required=True, help='current firmware Kd')
    p.add_argument('--label', default=None)
    p.add_argument('--set-gains', action='store_true',
                   help='send K: to push --kp/--ki/--kd into the firmware live '
                        'before testing (needs the K: command in firmware)')
    args = p.parse_args()

    speeds_mmps = [int(round(s * 1000)) for s in args.speeds]

    print("=" * 60)
    print("BAB IV TEST 6 - PID STEP RESPONSE DATA COLLECTION")
    print(f"Speeds   : {args.speeds} m/s  -> {speeds_mmps} mm/s")
    print(f"Reps     : {args.reps} per speed")
    print(f"Gains    : Kp={args.kp}  Ki={args.ki}  Kd={args.kd}")
    print("SAFETY: wheels off ground or clear space. ROS must be killed.")
    print("=" * 60)
    input("Press ENTER to start, Ctrl+C to cancel...")

    ser = connect_serial()

    if args.set_gains:
        print(f"\nPushing gains live: Kp={args.kp} Ki={args.ki} Kd={args.kd}")
        if not set_gains_live(ser, args.kp, args.ki, args.kd):
            print("Aborting: gains not confirmed. Flash the K: firmware or drop --set-gains.")
            ser.close()
            return

    raw_rows = []     # every sample, for the raw CSV
    metric_rows = []  # one row per (speed, rep, wheel)

    try:
        for target in speeds_mmps:
            for rep in range(1, args.reps + 1):
                print(f"\n--- {target} mm/s | rep {rep}/{args.reps} ---")
                samples = record_step(ser, target, args.duration)
                if not samples:
                    print("  no samples, skipping rep")
                    continue
                for s in samples:
                    raw_rows.append({'target_mmps': target, 'rep': rep, **s})

                for wheel in ('left', 'right'):
                    a = analyze(samples, target, wheel)
                    if a is None:
                        continue
                    metric_rows.append({
                        'target_mmps':   target,
                        'target_mps':    target / 1000.0,
                        'rep':           rep,
                        'wheel':         wheel,
                        'kp':            args.kp,
                        'ki':            args.ki,
                        'kd':            args.kd,
                        'rise_time_s':   a['rise_time_s'],
                        'overshoot_pct': a['overshoot_pct'],
                        'ss_speed_mmps': a['ss_speed_mmps'],
                        'ss_error_mmps': a['ss_error_mmps'],
                        'ss_error_pct':  a['ss_error_pct'],
                        'ss_rmse_mmps':  a['ss_rmse_mmps'],
                    })
                    print(f"  {wheel:5s}: rise={a['rise_time_s']}s "
                          f"OS={a['overshoot_pct']}% "
                          f"ss_err={a['ss_error_pct']}% "
                          f"RMSE={a['ss_rmse_mmps']}mm/s")

            # plot last rep of this speed
            last = [r for r in raw_rows if r['target_mmps'] == target]
            if last:
                last_rep = max(r['rep'] for r in last)
                png = U.timestamped_path('test6_pid_step',
                                         ext='png', label=f'{target}mmps')
                plot_step([r for r in last if r['rep'] == last_rep], target, png)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        for _ in range(10):
            send_velocity(ser, 0)
            time.sleep(0.05)
        ser.close()
        print("\nSerial closed, motors stopped.")

    if not metric_rows:
        print("No data collected.")
        return

    # save raw + metrics CSV
    raw_path = U.timestamped_path('test6_pid_raw', label=args.label)
    U.save_csv(raw_path,
               ['target_mmps', 'rep', 'time_s',
                'left_speed_mmps', 'right_speed_mmps'],
               [{'target_mmps': r['target_mmps'], 'rep': r['rep'],
                 'time_s': r['time_s'],
                 'left_speed_mmps': r['left_speed_mmps'],
                 'right_speed_mmps': r['right_speed_mmps']} for r in raw_rows])

    metric_path = U.timestamped_path('test6_pid_metrics', label=args.label)
    U.save_csv(metric_path,
               ['target_mmps', 'target_mps', 'rep', 'wheel',
                'kp', 'ki', 'kd', 'rise_time_s', 'overshoot_pct',
                'ss_speed_mmps', 'ss_error_mmps', 'ss_error_pct', 'ss_rmse_mmps'],
               metric_rows)

    # statistics per speed (both wheels pooled)
    print("\n" + "=" * 60)
    print("SUMMARY (mean/std/min/max across reps, both wheels)")
    print("=" * 60)
    for target in speeds_mmps:
        grp = [r for r in metric_rows if r['target_mmps'] == target]
        if not grp:
            continue
        print(f"\n  Target {target} mm/s ({target/1000:.2f} m/s):")
        U.print_summary_table('rise_time_s ',
                              U.summarize([r['rise_time_s'] for r in grp]), ' s')
        U.print_summary_table('overshoot_% ',
                              U.summarize([r['overshoot_pct'] for r in grp]), ' %')
        U.print_summary_table('ss_error_%  ',
                              U.summarize([r['ss_error_pct'] for r in grp]), ' %')
        U.print_summary_table('ss_RMSE mm/s',
                              U.summarize([r['ss_rmse_mmps'] for r in grp]), ' mm/s')

    print(f"\nRaw data    : {raw_path}")
    print(f"Metrics CSV : {metric_path}")
    print(f"Gains used  : Kp={args.kp} Ki={args.ki} Kd={args.kd}")


if __name__ == '__main__':
    main()
