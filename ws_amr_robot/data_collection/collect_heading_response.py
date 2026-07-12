#!/usr/bin/env python3
"""
collect_heading_response.py  --  Test PD heading controller (Test G-pra / heading response)
--------------------------------------------------------------------------------------------
Records the heading (theta) response of the robot WHILE it follows a Nav2 path, to prove
the PD heading controller in custom_path_controller.py damps the theta oscillation that a
raw Pure-Pursuit curvature law is prone to.

WHAT IT RECORDS (uniform time series, sampled at --rate Hz):
  t              seconds since recording start
  x, y           robot position (odom->base_link TF, fallback /odom)
  yaw_deg        actual robot heading theta        <- the response we care about
  v_cmd          commanded linear vel   (/cmd_vel_nav .linear.x)   [controller OUTPUT]
  w_cmd          commanded angular vel  (/cmd_vel_nav .angular.z)  <- shows PD action
  w_odom         measured angular vel   (/odom .twist.angular.z)
  path_yaw_deg   heading setpoint from /plan (angle along path ahead), NaN if no plan
  heading_err_deg wrap(path_yaw - yaw), NaN if no plan   <- tracking error to path

OSCILLATION METRICS (printed + in filename-companion .txt):
  w_sign_changes   number of zero-crossings of w_cmd -> direct oscillation count
  w_std            std-dev of w_cmd (rad/s)
  yaw_overshoot    max |heading_err| after first path alignment (deg)
  heading_rmse     RMSE of heading error to path (deg)

USAGE (run the nav stack first, then in separate terminals):
  # T1: hardware.launch.py     T2: navigation.launch.py
  # T3 (this):
  python3 collect_heading_response.py --label pd
  # T4: python3 send_nav_goal.py B4        (send a goal that forces a turn)
  # ...robot drives... then Ctrl+C here to stop, save CSV + plot.

  # For before/after comparison, set heading_kd:=0.0 (raw-PP-like) and re-run:
  #   ros2 param set /custom_path_controller heading_kd 0.0
  #   python3 collect_heading_response.py --label nopd
  # then: python3 plot_heading_compare.py --pd <pd.csv> --nopd <nopd.csv>

Output dir: ~/thesis_data/heading_test/
"""

import argparse
import math
import os
import signal
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import amr_test_utils as U


class HeadingResponseCollector(Node):
    def __init__(self, rate_hz, label):
        super().__init__('heading_response_collector')
        self.rate_hz = rate_hz
        self.label = label
        self.t0 = None
        self.rows = []

        # latest values
        self._odom = None            # (x, y, yaw, w_odom, v_odom) -- odom frame
        self._amcl = None             # (x, y, yaw) -- map frame, fallback pose
        self._cmd = (0.0, 0.0)       # (v_cmd, w_cmd)
        self._path = None            # list of (x, y), map frame (from /plan)
        # [alpha_rad, w_heading_radps, w_cmd_radps, w_odom_radps] from the
        # controller's own 'heading_debug' topic -- alpha is the literal PP
        # setpoint that feeds Kp/Kd, and w_odom is the wheel-encoder-diff
        # angular velocity (independent of AMCL/EKF). Same units as w_cmd
        # (rad, rad/s) so they can be compared directly, per the professor's
        # requested check.
        self._hdbg = (float('nan'), float('nan'), float('nan'), float('nan'))

        # TF for odom->base_link (preferred pose source, same as other tests)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 5)
        # controller output topic (see custom_controller.yaml: cmd_vel_topic)
        self.create_subscription(Twist, '/cmd_vel_nav', self._on_cmd, 20)
        self.create_subscription(Path, '/plan', self._on_plan, 5)
        self.create_subscription(Float64MultiArray, '/heading_debug', self._on_hdbg, 20)

        self.timer = self.create_timer(1.0 / rate_hz, self._sample)
        self.get_logger().info(
            f"Recording heading response (label='{label}', {rate_hz} Hz). "
            f"Send a nav goal now. Ctrl+C to stop & save.")

    def _on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = U.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._odom = (p.position.x, p.position.y, yaw,
                      msg.twist.twist.angular.z, msg.twist.twist.linear.x)

    def _on_cmd(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z)

    def _on_amcl(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self._amcl = (p.position.x, p.position.y, U.yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_hdbg(self, msg):
        d = msg.data
        if len(d) >= 4:
            self._hdbg = (d[0], d[1], d[2], d[3])

    def _on_plan(self, msg):
        self._path = [(ps.pose.position.x, ps.pose.position.y)
                      for ps in msg.poses] if msg.poses else None

    def _pose(self):
        """Pose in the MAP frame (map->base_link), to match /plan which Nav2
        publishes in 'map'. Using odom->base_link here would compare the path
        (map frame) against the robot position (odom frame) -- two frames
        that diverge over time (that's what AMCL corrects), producing bogus
        heading error. Falls back to /amcl_pose, then to /odom (odom frame,
        last resort -- only right if map and odom still coincide)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, U.yaw_from_quaternion(q.x, q.y, q.z, q.w))
        except (LookupException, ConnectivityException, ExtrapolationException):
            if self._amcl is not None:
                return self._amcl
            if self._odom is not None:
                return (self._odom[0], self._odom[1], self._odom[2])
            return None

    def _path_heading(self, x, y):
        """Heading of the path a bit ahead of the robot (the PD setpoint proxy).
        Returns NaN if no plan is available."""
        if not self._path or len(self._path) < 2:
            return float('nan')
        # nearest path index
        d2 = [(px - x) ** 2 + (py - y) ** 2 for px, py in self._path]
        i = min(range(len(d2)), key=lambda k: d2[k])
        j = min(i + 5, len(self._path) - 1)   # look ~5 poses ahead
        if j == i:
            j = min(i + 1, len(self._path) - 1)
        px, py = self._path[i]
        qx, qy = self._path[j]
        return math.atan2(qy - py, qx - px)

    def _sample(self):
        pose = self._pose()
        if pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        x, y, yaw = pose
        v_cmd, w_cmd = self._cmd
        w_odom = self._odom[3] if self._odom else float('nan')

        path_yaw = self._path_heading(x, y)
        if math.isnan(path_yaw):
            head_err = float('nan')
        else:
            head_err = U.wrap_to_pi(path_yaw - yaw)

        alpha_rad, w_heading, w_cmd_ctrl, w_odom_ctrl = self._hdbg

        self.rows.append({
            't': round(now - self.t0, 4),
            'x': round(x, 4),
            'y': round(y, 4),
            'yaw_deg': round(math.degrees(yaw), 3),
            'v_cmd': round(v_cmd, 4),
            'w_cmd': round(w_cmd, 4),
            'w_odom': round(w_odom, 4),
            'path_yaw_deg': round(math.degrees(path_yaw), 3) if not math.isnan(path_yaw) else float('nan'),
            'heading_err_deg': round(math.degrees(head_err), 3) if not math.isnan(head_err) else float('nan'),
            # From the controller's own 'heading_debug' topic -- the LITERAL
            # PP setpoint (alpha) that feeds Kp/Kd, in both units so nothing
            # gets mixed up when comparing to heading_err_deg above.
            'alpha_rad': round(alpha_rad, 5) if not math.isnan(alpha_rad) else float('nan'),
            'alpha_deg': round(math.degrees(alpha_rad), 3) if not math.isnan(alpha_rad) else float('nan'),
            'w_heading_radps': round(w_heading, 4) if not math.isnan(w_heading) else float('nan'),
            # w_odom above is /odom.twist.angular.z read directly by THIS
            # script; w_odom_ctrl is the same physical quantity but read by
            # the controller at its own tick -- should match closely; a
            # persistent mismatch would flag a subscription/timing bug.
            'w_odom_ctrl_radps': round(w_odom_ctrl, 4) if not math.isnan(w_odom_ctrl) else float('nan'),
        })


def compute_metrics(rows):
    """Oscillation + tracking metrics from the recorded rows."""
    if len(rows) < 3:
        return {}
    w = [r['w_cmd'] for r in rows]
    # zero-crossings of w_cmd, ignoring near-zero noise
    thresh = 0.02  # rad/s deadband to avoid counting noise
    sign_changes = 0
    last_sign = 0
    for val in w:
        if abs(val) < thresh:
            continue
        s = 1 if val > 0 else -1
        if last_sign != 0 and s != last_sign:
            sign_changes += 1
        last_sign = s

    n = len(w)
    mean_w = sum(w) / n
    w_std = math.sqrt(sum((v - mean_w) ** 2 for v in w) / n)

    herr = [r['heading_err_deg'] for r in rows
            if not math.isnan(r['heading_err_deg'])]
    if herr:
        heading_rmse = math.sqrt(sum(e * e for e in herr) / len(herr))
        # overshoot: after robot first gets within 20 deg of path, worst residual
        overshoot = 0.0
        aligned = False
        for e in herr:
            if abs(e) < 20.0:
                aligned = True
            if aligned:
                overshoot = max(overshoot, abs(e))
    else:
        heading_rmse = float('nan')
        overshoot = float('nan')

    # ---- professor's check: PP setpoint (alpha) vs actual wheel-encoder
    # heading response (w_odom_ctrl), same units (rad / rad/s), correlated.
    alpha_deg = [r['alpha_deg'] for r in rows if not math.isnan(r['alpha_deg'])]
    w_cmd_c   = [r['w_cmd'] for r in rows if not math.isnan(r.get('w_heading_radps', float('nan')))]
    w_odom_c  = [r['w_odom_ctrl_radps'] for r in rows if not math.isnan(r.get('w_odom_ctrl_radps', float('nan')))]

    alpha_mean_deg = round(sum(alpha_deg) / len(alpha_deg), 3) if alpha_deg else float('nan')
    alpha_std_deg  = (round(math.sqrt(sum((a - alpha_mean_deg) ** 2 for a in alpha_deg) / len(alpha_deg)), 3)
                      if alpha_deg else float('nan'))

    # Pearson correlation between commanded w and actual (wheel-diff) w --
    # should be strongly positive if the controller's output is actually
    # driving the measured rotation in the right direction/magnitude.
    w_corr = float('nan')
    if len(w_cmd_c) == len(w_odom_c) and len(w_cmd_c) > 2:
        n2 = len(w_cmd_c)
        mc = sum(w_cmd_c) / n2
        mo = sum(w_odom_c) / n2
        cov = sum((w_cmd_c[i] - mc) * (w_odom_c[i] - mo) for i in range(n2))
        sc = math.sqrt(sum((v - mc) ** 2 for v in w_cmd_c))
        so = math.sqrt(sum((v - mo) ** 2 for v in w_odom_c))
        if sc > 1e-9 and so > 1e-9:
            w_corr = round(cov / (sc * so), 3)

    return {
        'samples': n,
        'duration_s': round(rows[-1]['t'], 2),
        'w_sign_changes': sign_changes,
        'w_std': round(w_std, 4),
        'heading_rmse_deg': round(heading_rmse, 2) if not math.isnan(heading_rmse) else float('nan'),
        'yaw_overshoot_deg': round(overshoot, 2) if not math.isnan(overshoot) else float('nan'),
        # -- PP setpoint (alpha) vs actual (wheel-diff) checks --
        'alpha_mean_deg': alpha_mean_deg,   # should be ~0 on a straight path
        'alpha_std_deg': alpha_std_deg,
        'w_cmd_vs_w_odom_corr': w_corr,     # should be strongly positive (~>0.7) if PD is actually correcting what it measures
    }


def save_and_plot(rows, label, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = f'heading_{label}_{ts}'
    csv_path = os.path.join(out_dir, base + '.csv')

    fieldnames = ['t', 'x', 'y', 'yaw_deg', 'v_cmd', 'w_cmd', 'w_odom',
                  'path_yaw_deg', 'heading_err_deg',
                  'alpha_rad', 'alpha_deg', 'w_heading_radps', 'w_odom_ctrl_radps']
    U.save_csv(csv_path, fieldnames, rows)
    print(f"\nCSV saved: {csv_path}")

    metrics = compute_metrics(rows)
    print("\n" + "=" * 55)
    print(f"  HEADING RESPONSE METRICS  (label='{label}')")
    print("=" * 55)
    for k, v in metrics.items():
        print(f"  {k:20s}: {v}")
    print("  (lower w_sign_changes / w_std / overshoot = better damping)")

    # write metrics next to csv
    with open(os.path.join(out_dir, base + '_metrics.txt'), 'w') as f:
        f.write(f"label={label}\n")
        for k, v in metrics.items():
            f.write(f"{k}={v}\n")

    # plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; CSV saved, no plot.")
        return

    t = [r['t'] for r in rows]
    yaw = [r['yaw_deg'] for r in rows]
    pyaw = [r['path_yaw_deg'] for r in rows]
    wc = [r['w_cmd'] for r in rows]
    xs = [r['x'] for r in rows]
    ys = [r['y'] for r in rows]

    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(11, 10))

    a1.plot(t, yaw, color='#2196F3', lw=1.5, label='actual heading (theta)')
    if any(not math.isnan(p) for p in pyaw):
        a1.plot(t, pyaw, color='#F44336', lw=1.2, ls='--', label='path heading (setpoint)')
    a1.set_ylabel('heading (deg)')
    a1.set_title(f"Heading response — label='{label}'  "
                 f"(sign-changes={metrics.get('w_sign_changes')}, "
                 f"w_std={metrics.get('w_std')})")
    a1.legend(); a1.grid(alpha=0.3)

    a2.plot(t, wc, color='#4CAF50', lw=1.3)
    a2.axhline(0, color='k', lw=0.6)
    a2.set_ylabel('w_cmd (rad/s)')
    a2.set_xlabel('time (s)')
    a2.set_title('Angular velocity command — oscillation shows as sign changes')
    a2.grid(alpha=0.3)

    a3.plot(xs, ys, color='#673AB7', lw=1.5)
    a3.plot(xs[0], ys[0], 'go', label='start')
    a3.plot(xs[-1], ys[-1], 'rs', label='end')
    a3.set_xlabel('x (m)'); a3.set_ylabel('y (m)')
    a3.set_title('Path taken (odom frame)')
    a3.axis('equal'); a3.legend(); a3.grid(alpha=0.3)

    fig.tight_layout()
    png = os.path.join(out_dir, base + '.png')
    fig.savefig(png, dpi=150, bbox_inches='tight')
    print(f"Plot saved: {png}")


def main():
    ap = argparse.ArgumentParser(description='Record PD heading-controller response during navigation')
    ap.add_argument('--label', default='pd',
                    help="run label, e.g. 'pd' or 'nopd' (default: pd)")
    ap.add_argument('--rate', type=float, default=25.0,
                    help='sampling rate Hz (default 25 = control freq)')
    ap.add_argument('--out-dir', default=None,
                    help='output dir (default ~/thesis_data/heading_test)')
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.expanduser('~/thesis_data/heading_test')

    rclpy.init()
    node = HeadingResponseCollector(args.rate, args.label)

    stop = {'flag': False}

    def handler(sig, frame):
        stop['flag'] = True
    signal.signal(signal.SIGINT, handler)

    try:
        while rclpy.ok() and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        rows = node.rows
        node.destroy_node()
        rclpy.shutdown()
        if rows:
            save_and_plot(rows, args.label, out_dir)
        else:
            print("\nNo samples recorded — was the nav stack running and a goal sent?")


if __name__ == '__main__':
    main()
