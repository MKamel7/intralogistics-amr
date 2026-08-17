#!/usr/bin/env python3
"""Grab the RViz window, and stamp every grab with the SIMULATED time.

RViz cannot be filmed the way the world can. There is no camera sensor inside
it; it is an X window, and the only way to record it is to grab pixels. That is
the technique this project already got wrong once.

What made the screen grab wrong was never the grabbing. It was that a wall
clock sample rate was encoded as though it were uniform and simulated: 4.3
frames per second played at 10, so the demo ran 2.3 times fast. The pixels were
fine. The timeline was invented.

So grab the window, but read the clock at the same moment and keep it. Each
frame then carries the simulated time it depicts, exactly like a camera sensor
frame, and `film.hold_indices` resamples it onto a uniform grid the same way.
An RViz segment cut like this runs at the same speed as the footage beside it,
which is the property that lets the two be shown in one video at all.

It helps that the simulator is slow while filming. At a real time factor near
0.1, a grab every 200 ms of wall clock is a sample every 20 ms of simulated
time, so there is more than enough to fill 30 fps of output.

The window is found by name rather than by clicking, so this does not depend on
where anything was left on screen. `import -window <id>` reads the window's own
contents, which is why it works under XWayland where a root-window grab returns
solid black: that black frame was once reported as a working recording on the
strength of the file existing.
"""

import argparse
import os
import subprocess
import sys
import time


def window_id(pattern):
    """The X id of the first window whose title matches, or None."""
    try:
        out = subprocess.run(['wmctrl', '-l'], capture_output=True, text=True,
                             timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    for line in out.splitlines():
        if pattern.lower() in line.lower():
            return line.split()[0]
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--window', default='RViz', help='window title substring')
    ap.add_argument('--duration', type=float, default=240.0,
                    help='wall clock seconds to grab for')
    ap.add_argument('--interval', type=float, default=0.20,
                    help='wall clock seconds between grabs')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    outdir = args.out or os.path.join(os.environ.get('SCRATCH', '/tmp'),
                                      'film_rviz')
    os.makedirs(outdir, exist_ok=True)

    wid = window_id(args.window)
    if wid is None:
        print(f'no window matching {args.window!r}; is RViz up?')
        return 1
    print(f'grabbing window {wid} every {args.interval:.2f} s for '
          f'{args.duration:.0f} s into {outdir}')

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    class Clock(Node):
        def __init__(self):
            super().__init__('film_rviz', parameter_overrides=[
                Parameter('use_sim_time', value=True)])

        def sim_now(self):
            # Spin briefly so /clock is current rather than whatever arrived
            # when the node was constructed.
            rclpy.spin_once(self, timeout_sec=0.05)
            t = self.get_clock().now().nanoseconds
            return t / 1e9

    rclpy.init()
    node = Clock()
    # Let the clock subscription settle. A first grab stamped with 0.0 would
    # anchor the whole timeline at the epoch.
    for _ in range(40):
        if node.sim_now() > 1.0:
            break
        time.sleep(0.1)

    stamps = []
    stop = time.time() + args.duration
    i = 0
    blank = 0
    while time.time() < stop:
        t = node.sim_now()
        path = os.path.join(outdir, f'{i:05d}.png')
        r = subprocess.run(['import', '-window', wid, path],
                           capture_output=True)
        if r.returncode == 0 and os.path.exists(path):
            if os.path.getsize(path) < 20000:
                # A nearly empty PNG is the XWayland black frame. Count it and
                # say so at the end rather than shipping a black segment.
                blank += 1
            stamps.append((i, t))
            i += 1
        time.sleep(args.interval)

    with open(os.path.join(outdir, 'stamps.csv'), 'w') as fh:
        fh.write('frame,sim_t\n')
        for n, t in stamps:
            fh.write(f'{n},{t:.3f}\n')

    node.destroy_node()
    rclpy.shutdown()
    if not stamps:
        print('no frames grabbed')
        return 1
    span = stamps[-1][1] - stamps[0][1]
    print(f'{len(stamps)} grabs spanning {span:.1f} s of simulated time '
          f'({len(stamps) / max(span, 1e-6):.1f} per simulated second)')
    if blank:
        print(f'WARNING: {blank} of {len(stamps)} grabs are under 20 KB and '
              f'are probably blank; check one before using this')
    return 0


if __name__ == '__main__':
    sys.exit(main())
