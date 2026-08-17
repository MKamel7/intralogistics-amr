#!/usr/bin/env python3
"""Cut the demo video: pick the beats by rule, caption them, assemble.

WHY THE CUT IS A PROGRAM AND NOT A LIST OF TIMESTAMPS

`film.py` records and conforms. What it does not do is decide what the video
says, and the first two attempts at this project's demo were both lost there
rather than in the footage: segments were chosen by eye from a scrub bar, one
of them showed a vehicle standing still under a protective stop and another
began after the interesting event had finished. Both looked deliberate.

So the structure lives here, as data, and every beat is filled by a QUERY over
the recording's own per-frame logs:

  - an establishing beat asks for the window where the vehicle is best framed
    and actually moving (`film.report`)
  - the safety beat asks for a window in which THE COLLISION MONITOR FIRED
    (`film.events`), because that is the only beat that makes a claim, and a
    claim about the monitor has to be answered by the monitor

If a beat cannot be filled, it is DROPPED and the assembly says so. That is the
whole point of doing it this way: the previous cut filled a safety beat with
footage that had no safety event in it, and nothing in the process objected.

WHAT THE CAPTIONS MAY SAY

A caption is a claim, so a caption is only allowed to state something read out
of the logs at the frames it is shown over. The "PROTECTIVE STOP" badge is
displayed exactly over the frames whose logged action is `stop`, computed from
the sim stamps and the conform grid, never placed by hand. The closing card
quotes the results table in README.md, with the V numbers that produced each
figure, including the one that was not delivered.
"""

import argparse
import csv
import importlib.util
import json
import os
import subprocess
import sys

FILM = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'film.py')

W, H = 1280, 720
RATE = 30

BG = (14, 17, 22)
FG = (232, 237, 242)
MUTED = (154, 167, 178)
ACCENT = (127, 209, 185)
WARN = (242, 180, 65)

SANS = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
SANS_B = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
MONO = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'

# --- the structure ---------------------------------------------------------
#
# Read this list top to bottom and you have the video. Cards carry the
# argument, beats carry the evidence, and each beat names the rule that fills
# it rather than a timestamp.

STRUCTURE = [
    dict(kind='card', seconds=3.5,
         head='Intralogistics AMR',
         sub='Autonomous transport in a warehouse shared with people',
         body=['MP-400 class  ·  ROS 2 Jazzy  ·  Gazebo Harmonic  ·  Nav2']),

    dict(kind='beat', rule='motion', seconds=9.0, source=0,
         label='Autonomous survey',
         note='driving frontier goals to build its own map',
         # Ranked, then looked at: the top scoring window hides the vehicle
         # behind a pallet stack, which every gate scores as a clear view.
         pin='pass:59.0'),

    dict(kind='card', seconds=3.0,
         head='It plans, it does not follow a line',
         sub='Nav2: global plan, local costmap, MPPI candidate trajectories'),

    dict(kind='beat', rule='rviz', seconds=8.0,
         label='Nav2 planning',
         note='the robot\'s own view, resampled to the same clock'),

    dict(kind='card', seconds=3.0,
         head='It carries the load',
         sub='100 kg, unsecured, measured over a full duty cycle'),

    dict(kind='beat', rule='motion', seconds=8.0, distinct=True, source=1,
         label='Transport cycle',
         note='carrying an unsecured 100 kg load',
         fallback_label='Surveying, second view',
         fallback_note='a different tripod, no load aboard'),

    dict(kind='card', seconds=4.0,
         head='It gives people room',
         sub='A human aware costmap layer keeps the vehicle out of personal '
             'space instead of waiting until a field trips',
         body=['Time spent in intimate space, layer on: 5.00 %,',
               'against 7.33 % with it off (V-64).']),

    # Source 2: the only recording whose frame log carries the distance to
    # the nearest person, which is what makes this beat selectable at all.
    dict(kind='beat', rule='avoid', seconds=8.0, source=2,
         label='Passing a person',
         note='keeps moving, leaves room'),

    dict(kind='card', seconds=6.0,
         head='Measured, not asserted',
         sub='From the results table in README.md, measured on the '
             'datasheet-sized test track',
         table=[('contacts the vehicle drove into', '0 in 248 000 samples',
                 'V-51'),
                ('deepest a person reached inside the footprint', '-0.100 m',
                 'V-49'),
                ('localisation error, driving', 'p50 0.027 m', 'V-37'),
                ('unsecured 100 kg load over a duty cycle', '0.0 mm slide',
                 'V-61'),
                ('parked accuracy at a station', 'median 117 mm', 'V-62')]),
]


def film():
    """Import film.py as a module: it is a tool, not a package."""
    spec = importlib.util.spec_from_file_location('film', FILM)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def font(size, bold=False, mono=False):
    from PIL import ImageFont
    return ImageFont.truetype(MONO if mono else (SANS_B if bold else SANS),
                              size)


def wrap(draw, text, fnt, width):
    words, lines, cur = text.split(), [], ''
    for word in words:
        trial = (cur + ' ' + word).strip()
        if draw.textlength(trial, font=fnt) <= width:
            cur = trial
        else:
            lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    return lines


def card(spec, dest):
    """Render one title card.

    Laid out in two passes so the block sits in the MIDDLE of the frame. Drawn
    from a fixed top margin the short cards left a third of the screen empty
    below them and read as though the render had been cut off.
    """
    from PIL import Image, ImageDraw
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    x = 96
    f_head = font(50, bold=True)
    f_sub = font(27)
    f_body = font(23, mono=True)
    head = wrap(d, spec['head'], f_head, W - 2 * x)
    sub = wrap(d, spec.get('sub', ''), f_sub, W - 2 * x - 60)
    body = spec.get('body', [])
    total = (len(head) * 62 + 10 + len(sub) * 36 + 26 + len(body) * 32
             + len(spec.get('table', [])) * 34)
    y = max(130, (H - total) // 2)
    d.rectangle([x, y - 46, x + 64, y - 40], fill=ACCENT)
    for line in head:
        d.text((x, y), line, font=f_head, fill=FG)
        y += 62
    y += 10
    for line in sub:
        d.text((x, y), line, font=f_sub, fill=MUTED)
        y += 36
    y += 26
    for line in body:
        d.text((x, y), line, font=f_body, fill=FG)
        y += 32
    if spec.get('table'):
        f_k = font(22, mono=True)
        for key, val, ref in spec['table']:
            d.text((x, y), key, font=f_k, fill=MUTED)
            colour = WARN if val.isupper() else ACCENT
            vw = d.textlength(val, font=f_k)
            rw = d.textlength(ref, font=f_k)
            d.text((W - x - rw, y), ref, font=f_k, fill=MUTED)
            d.text((W - x - rw - 24 - vw, y), val, font=f_k, fill=colour)
            y += 34
    img.save(dest)
    return dest


def lower_third(text, note, dest):
    """A caption strip, rendered with alpha so ffmpeg can overlay it."""
    from PIL import Image, ImageDraw
    img = Image.new('RGBA', (W, 96), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, W, 96], fill=(14, 17, 22, 190))
    d.rectangle([64, 26, 70, 70], fill=ACCENT)
    d.text((92, 22), text, font=font(30, bold=True), fill=FG)
    d.text((92, 58), note, font=font(21), fill=MUTED)
    img.save(dest)
    return dest


def badge(dest):
    """The monitor badge. Shown only over frames the monitor called a stop."""
    from PIL import Image, ImageDraw
    img = Image.new('RGBA', (520, 78), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, 520, 78], fill=(70, 20, 20, 215))
    d.rectangle([0, 0, 8, 78], fill=(232, 90, 80))
    d.text((28, 12), 'PROTECTIVE STOP', font=font(28, bold=True),
           fill=(255, 226, 222))
    d.text((28, 46), 'collision_monitor · protective field',
           font=font(18, mono=True), fill=(240, 190, 185))
    img.save(dest)
    return dest


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode:
        sys.stderr.write(' '.join(cmd[:6]) + ' ...\n' + p.stderr[-700:] + '\n')
    return p.returncode == 0


def encode_card(png, seconds, dest):
    return run(['ffmpeg', '-y', '-v', 'error', '-loop', '1', '-i', png,
                '-t', f'{seconds}', '-r', str(RATE),
                '-vf', f'fade=t=in:st=0:d=0.4,'
                       f'fade=t=out:st={max(0.0, seconds - 0.5):.2f}:d=0.5,'
                       f'format=yuv420p',
                '-c:v', 'libx264', '-preset', 'slow', '-crf', '20',
                dest]) and dest


def encode_beat(clip, strip, seconds, dest, stops=()):
    """Normalise a conformed clip and burn its caption on.

    NO `-r` HERE, and none in the concat either. The clip is already on a
    uniform grid at the output rate from `film.conform`, and re-timing it at
    this stage would undo the one property the recording exists to have.
    """
    inputs = ['-i', clip, '-i', strip]
    chain = (f'[0:v]scale={W}:{H}:force_original_aspect_ratio=decrease,'
             f'pad={W}:{H}:(ow-iw)/2:(oh-ih)/2[v];'
             f'[v][1:v]overlay=0:{H - 96}[c]')
    last = 'c'
    if stops:
        inputs += ['-i', badge(os.path.join(os.path.dirname(dest),
                                            'badge.png'))]
        # Exactly the intervals whose frames the monitor called a stop.
        expr = '+'.join(f'between(t,{a:.3f},{b:.3f})' for a, b in stops)
        chain += (f';[c][2:v]overlay=64:64:enable=\'{expr}\'[s]')
        last = 's'
    chain += (f';[{last}]fade=t=in:st=0:d=0.4,'
              f'fade=t=out:st={max(0.0, seconds - 0.5):.2f}:d=0.5,'
              f'format=yuv420p[o]')
    return run(['ffmpeg', '-y', '-v', 'error'] + inputs +
               ['-filter_complex', chain, '-map', '[o]',
                '-c:v', 'libx264', '-preset', 'slow', '-crf', '20',
                dest]) and dest


def concat(parts, dest):
    lst = dest + '.txt'
    with open(lst, 'w') as fh:
        for p in parts:
            fh.write(f"file '{os.path.abspath(p)}'\n")
    ok = run(['ffmpeg', '-y', '-v', 'error', '-f', 'concat', '-safe', '0',
              '-i', lst, '-c:v', 'libx264', '-preset', 'slow', '-crf', '21',
              '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-an', dest])
    os.remove(lst)
    return ok


def busiest_window(srcdir, rows, want, skip=0.0):
    """Start offset of the `want` second window with the most on-screen change.

    Change is measured on heavily downsampled greyscale, because the question
    is whether the picture is moving, not what it shows.
    """
    import cv2
    import numpy as np
    ts = [float(r['sim_t']) for r in rows]
    small = []
    for r in rows:
        img = cv2.imread(os.path.join(srcdir, f"{int(r['frame']):05d}.png"),
                         cv2.IMREAD_GRAYSCALE)
        small.append(None if img is None
                     else cv2.resize(img, (64, 40)).astype(np.int16))
    diff = [0.0]
    for a, b in zip(small, small[1:]):
        diff.append(0.0 if a is None or b is None
                    else float(np.abs(b - a).mean()))
    best, best_ss = -1.0, skip
    for lo in range(len(ts)):
        if ts[lo] - ts[0] < skip:
            continue
        if ts[lo] - ts[0] + want > ts[-1] - ts[0]:
            break
        hi = lo
        while hi + 1 < len(ts) and ts[hi + 1] <= ts[lo] + want:
            hi += 1
        if hi - lo < 4:
            continue
        score = sum(diff[lo:hi + 1]) / (hi - lo)
        if score > best:
            best, best_ss = score, ts[lo] - ts[0]
    return best_ss


def rviz_clip(srcdir, ss, seconds, rate, dest, f):
    """Conform a window of RViz grabs onto the same uniform grid as the rest.

    RViz has no camera sensor in it; it is an X window and the only way to
    record it is to grab pixels, which is the technique this project got wrong
    once. What made that wrong was never the grabbing, it was encoding a wall
    clock sample rate as though it were uniform and simulated. So the grabs
    carry the SIMULATED time they depict and are resampled here by the same
    `hold_indices` the camera footage uses. An RViz segment then runs at the
    same speed as the shot beside it, which is what lets the two sit in one
    video.

    The window is cropped to the 3D view. The panels are RViz's own furniture
    and say nothing about the robot.
    """
    import cv2
    with open(os.path.join(srcdir, 'stamps.csv')) as fh:
        rows = [r for r in csv.DictReader(fh) if r.get('sim_t')]
    stamps = [float(r['sim_t']) for r in rows]
    names = [os.path.join(srcdir, f"{int(r['frame']):05d}.png") for r in rows]
    if len(stamps) < 4:
        return None, 'fewer than four grabs'
    idx = f.hold_indices(stamps, ss, seconds, rate)
    if not idx:
        return None, 'no frames in that window'
    out, cache = None, {}
    for i in idx:
        if i not in cache:
            img = cv2.imread(names[i])
            if img is None:
                continue
            h, w = img.shape[:2]
            # THE MAP, not the application. Cropping to the 3D view still left
            # RViz's toolbar across the top and the map itself a narrow strip
            # in a field of empty grid, because a top down orthographic view of
            # this building is portrait and the window is not. These bounds are
            # measured off a grab and frame the map with a margin. The clip
            # keeps its own aspect ratio and is letterboxed by encode_beat,
            # rather than being stretched to 16:9.
            img = img[int(0.06 * h):int(0.89 * h),
                      int(0.407 * w):int(0.814 * w)]
            cache[i] = img
        if i not in cache:
            continue
        if out is None:
            hh, ww = cache[i].shape[:2]
            out = cv2.VideoWriter(dest, cv2.VideoWriter_fourcc(*'mp4v'),
                                  float(rate), (ww, hh))
        out.write(cache[i])
    if out is None:
        return None, 'no readable grabs'
    out.release()
    return dest, None


def verify_pin(rows, ts, ss, seconds, rule, min_px, max_px):
    """Check a hand-picked window against the same gates the query applies.

    A PIN IS THE RESULT OF LOOKING, and looking is the instrument that caught
    what the query could not: two of the four cameras in this recording are
    partly blocked by a rack upright or a stack of pallets, and the frustum test
    scores those frames as a clear view because it knows where the vehicle is
    and not what is in front of it.

    So a pin is allowed to overrule the RANKING. It is not allowed to overrule
    the FACTS. A pinned safety beat with no logged stop inside it, or a pinned
    shot with the vehicle out of frame, is rejected here rather than quietly
    filmed, which is the failure the whole selector exists to prevent.
    """
    t0 = ts[0] + ss
    win = [r for r, t in zip(rows, ts) if 0.0 <= t - t0 <= seconds]
    if len(win) < max(4, seconds * 3):
        return None, f'only {len(win)} frames in the window'
    vis = [r for r in win if r['visible'] == '1' and r['px_across']]
    if len(vis) < 0.6 * len(win):
        return None, (f'vehicle in frame for only '
                      f'{100.0 * len(vis) / len(win):.0f}% of it')
    px = sum(float(r['px_across']) for r in vis) / len(vis)
    if not min_px <= px <= max_px:
        return None, f'vehicle averages {px:.0f} px, outside {min_px}-{max_px}'
    sp = sum(float(r['speed_mps']) for r in vis if r['speed_mps']) / \
        max(1, len([r for r in vis if r['speed_mps']]))
    if rule == 'event':
        firing = [r for r in win if (r.get('action') or 'clear') == 'stop']
        if not firing:
            return None, 'no logged protective stop inside the window'
        # AND THE VEHICLE MUST ACTUALLY STOP. A pin that passed on the logged
        # action alone is how a PROTECTIVE STOP badge came to sit over a
        # vehicle driving through frame at 0.75 m/s: the monitor asserted stop
        # on thirteen frames and the speed never fell below 0.44 m/s.
        rest = [r for r in win if float(r['speed_mps'] or 9.0) < 0.05]
        if len(rest) < 3:
            return None, (f'monitor fired but the vehicle never came to rest '
                          f'({len(rest)} frame(s) under 0.05 m/s)')
        poly = next((r['polygon'] for r in firing if r['polygon']), '')
        return (f'stop on the {poly} field, {len(rest)} frames at rest, '
                f'{px:.0f} px across, {100.0 * len(vis) / len(win):.0f}% '
                f'in frame'), None
    return (f'{px:.0f} px across, {sp:.2f} m/s, '
            f'{100.0 * len(vis) / len(win):.0f}% in frame'), None


def stop_intervals(rows, ts, ss, seconds):
    """Output-clip times over which the logged action is `stop`.

    The clip starts at `ss` seconds of simulated time after the camera's first
    frame and runs on a uniform grid, so a frame's position in the OUTPUT is
    its sim stamp minus the clip's start stamp. Nothing here is placed by hand.

    THE BADGE IS HELD LONGER THAN THE EVENT, deliberately and by a stated
    amount. Stops are often three or four frames of a camera running at a dozen
    Hz, which at true speed is a tenth of a second: real, logged, and invisible.
    So it appears LEAD_IN before the first stop frame and stays TAIL after the
    last, with a floor of MIN_SHOWN. The card that introduces the shot says so,
    because a badge that outlives its event without saying it does is the same
    class of lie as a video that plays at the wrong speed.
    """
    LEAD_IN, TAIL, MIN_SHOWN = 0.15, 0.6, 1.0
    t0 = ts[0] + ss
    out, run_start = [], None
    for r, t in zip(rows, ts):
        inside = 0.0 <= t - t0 <= seconds
        firing = (r.get('action') or 'clear') == 'stop'
        if inside and firing and run_start is None:
            run_start = t - t0
        elif run_start is not None and (not firing or not inside):
            out.append((max(0.0, run_start - LEAD_IN),
                        min(seconds, t - t0 + TAIL)))
            run_start = None
    if run_start is not None:
        out.append((max(0.0, run_start - LEAD_IN), seconds))
    out = [(a, min(seconds, max(b, a + MIN_SHOWN))) for a, b in out]
    # Merge intervals that touch, so the badge does not flicker between two
    # frames of the same stop.
    merged = []
    for a, b in sorted(out):
        if merged and a <= merged[-1][1] + 0.2:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    return merged


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', action='append', default=None,
                    help='a recording directory written by film.py. Repeat it '
                         'to cut across several: the survey and the transport '
                         'mission are different phases of the run and a beat '
                         'is pinned to the one it claims to show.')
    ap.add_argument('--dest', default='docs/media/demo.mp4')
    ap.add_argument('--rviz', default=None,
                    help='a tools/film_rviz.py capture directory, for the '
                         'Nav2 beat')
    ap.add_argument('--min-px', type=float, default=110.0)
    ap.add_argument('--max-px', type=float, default=680.0)
    ap.add_argument('--min-speed', type=float, default=0.25)
    ap.add_argument('--rviz-skip', type=float, default=5.0,
                    help='seconds to ignore at the start of the RViz capture, '
                         'where the goal arrow is placed by hand')
    ap.add_argument('--near', type=float, default=2.0,
                    help='how close a pass has to be to count as one')
    args = ap.parse_args()

    f = film()
    dirs = args.out or [os.path.join(os.environ.get('SCRATCH', '/tmp'),
                                     'film')]
    work = os.path.join(dirs[0], 'cut')
    os.makedirs(work, exist_ok=True)
    sources = []
    for path in dirs:
        got = f.logs(path) if os.path.isdir(path) else {}
        if got:
            sources.append((path, got))
        else:
            print(f'no recording in {path}; ignoring it')
    if not sources:
        print('no recordings to cut from')
        return 1

    parts, manifest, used_cams, used_spans = [], [], [], []

    def overlaps(src, cam, ss, seconds):
        for s, c, a, b in used_spans:
            if s == src and c == cam and ss < b and a < ss + seconds:
                return True
        return False

    def source_for(spec):
        """The recording a beat is pinned to, or the only one there is.

        A beat names the PHASE it claims to show. If that recording is missing
        the beat falls back rather than vanishing, but it says so, because a
        transport caption over survey footage is exactly the kind of quiet
        substitution this file exists to prevent.
        """
        want = spec.get('source', 0)
        if want < len(sources):
            return sources[want], False
        return sources[0], True

    for i, spec in enumerate(STRUCTURE):
        tag = os.path.join(work, f'{i:02d}')
        if spec['kind'] == 'card':
            png = card(spec, tag + '.png')
            part = encode_card(png, spec['seconds'], tag + '.mp4')
            if part:
                parts.append(part)
                manifest.append(dict(beat=i, kind='card', head=spec['head']))
            continue

        want = spec['seconds']
        if spec['rule'] == 'rviz':
            if not args.rviz or not os.path.isdir(args.rviz):
                print(f'beat {i}: no RViz capture given; dropping the Nav2 '
                      f'beat rather than showing something else')
                continue
            with open(os.path.join(args.rviz, 'stamps.csv')) as fh:
                rr = [r for r in csv.DictReader(fh) if r.get('sim_t')]
            span = float(rr[-1]['sim_t']) - float(rr[0]['sim_t']) if rr else 0.0
            if span < want:
                print(f'beat {i}: RViz capture spans only {span:.1f} s; '
                      f'dropping it')
                continue
            # THE WINDOW WHERE SOMETHING HAPPENS. Taking a fixed position in
            # the capture is a guess about when the robot was driving: the
            # middle showed a half explored building, and the end, once the
            # goals had finished, is a complete map with a parked robot and
            # nothing moving on it. Both are the same mistake the shot
            # selector exists to prevent, made about a different recording.
            #
            # So the frames are asked directly. The window whose consecutive
            # grabs differ the most is the one with a plan being redrawn and a
            # robot crossing the map, and it is found the same way as every
            # other beat here: by measuring, not by choosing a timestamp.
            # SKIP THE OPENING, where the goal is placed. The 2D Goal Pose
            # arrow is a person reaching into the scene with a mouse, and a
            # demo of autonomous navigation should not open on the hand that
            # set the destination. The plan and the drive that follow are the
            # part that is about the robot.
            ss = busiest_window(args.rviz, rr, want, args.rviz_skip)
            clip, bad = rviz_clip(args.rviz, ss, want, RATE,
                                  tag + '_raw.mp4', f)
            if bad:
                print(f'beat {i}: RViz beat dropped ({bad})')
                continue
            strip = lower_third(spec['label'], spec['note'],
                                tag + '_strip.png')
            part = encode_beat(clip, strip, want, tag + '.mp4')
            if not part:
                print(f'beat {i}: RViz encode failed; dropping it')
                continue
            parts.append(part)
            manifest.append(dict(beat=i, kind='beat', rule='rviz',
                                 label=spec['label'], ss=round(ss, 1),
                                 seconds=want,
                                 why=f'{len(rr)} grabs spanning {span:.0f} s'))
            print(f'beat {i}: {spec["label"]:22s} rviz   +{ss:6.1f}s  '
                  f'{len(rr)} grabs over {span:.0f} s')
            continue

        (srcdir, srclogs), fell_back = source_for(spec)
        pin = None if fell_back else spec.get('pin')
        label, note = spec['label'], spec['note']
        if fell_back:
            # THE CAPTION IS RENAMED, not merely qualified. Annotating a beat
            # still titled "Transport cycle" with a note that it came from the
            # survey leaves the claim standing in the largest text on screen.
            label = spec.get('fallback_label', label)
            note = spec.get('fallback_note', note + ' (phase not recorded)')
            print(f'beat {i}: recording {spec.get("source")} is missing; '
                  f'using {srcdir} and retitling the beat "{label}"')
        if pin:
            cam, ss = pin.split(':')[0], float(pin.split(':')[1])
            if cam not in srclogs:
                print(f'beat {i}: pinned camera {cam} is not in {srcdir}; '
                      f'dropping the beat')
                continue
            rows = srclogs[cam]
            ts = [float(r['sim_t']) for r in rows]
            why, bad = verify_pin(rows, ts, ss, want, spec['rule'],
                                  args.min_px if spec['rule'] == 'motion'
                                  else 90.0, args.max_px)
            if bad:
                print(f'beat {i}: pinned {pin} fails its own gate ({bad}); '
                      f'dropping it rather than filming it anyway')
                continue
            why = why + ', confirmed by eye'
            stops = (stop_intervals(rows, ts, ss, want)
                     if spec['rule'] == 'event' else ())
        elif spec['rule'] == 'avoid':
            cands = f.avoids(srcdir, want, args.min_px, args.max_px,
                             args.near, 0.05)
            cands = [c for c in cands if not overlaps(srcdir, c[1], c[2], want)]
            if not cands:
                print(f'beat {i}: no close pass that kept moving; dropping '
                      f'the beat rather than showing an ordinary drive-by')
                continue
            _, cam, ss, px, sp, closest = cands[0]
            why = (f'passed a person at {closest:.2f} m without stopping, '
                   f'mean {sp:.2f} m/s, {px:.0f} px across')
            note = f'{note}, closest {closest:.2f} m'
            stops = ()
        elif spec['rule'] == 'motion':
            cands = f.report(srcdir, RATE, want, args.min_px,
                             args.min_speed, args.max_px)
            cands = [(s, n, ss, px, sp, fr) for s, n, ss, px, sp, fr in cands
                     if not overlaps(srcdir, n, ss, want)]
            if spec.get('distinct'):
                cands = [c for c in cands if c[1] not in used_cams] or cands
            if not cands:
                print(f'beat {i}: no motion window passed; dropping it')
                continue
            score, cam, ss, px, sp, frac = cands[0]
            why = f'{px:.0f} px across, {sp:.2f} m/s, {frac * 100:.0f}% in frame'
            stops = ()
        else:
            evs = f.events(srcdir, want, 90.0, args.max_px, 4.0)
            evs = [e for e in evs if e[6] == 'stop'
                   and not overlaps(srcdir, e[4], e[5], want)]
            if not evs:
                print(f'beat {i}: the run produced no protective stop in '
                      f'frame; dropping the safety beat rather than '
                      f'illustrating it with something else')
                continue
            _, halt, held, frac, cam, ss, act, poly, px = evs[0]
            why = (f'{act} on the {poly} field, vehicle at rest {halt:.1f} s, '
                   f'monitor asserting {held:.1f} s, {px:.0f} px across')
            rows = srclogs[cam]
            ts = [float(r['sim_t']) for r in rows]
            stops = stop_intervals(rows, ts, ss, want)

        clip = f.conform(srcdir, cam, ss, want, RATE, tag + '_raw.mp4')
        if not clip:
            print(f'beat {i}: conform failed; dropping it')
            continue
        strip = lower_third(label, note, tag + '_strip.png')
        part = encode_beat(clip, strip, want, tag + '.mp4', stops)
        if not part:
            print(f'beat {i}: encode failed; dropping it')
            continue
        parts.append(part)
        used_cams.append(cam)
        used_spans.append((srcdir, cam, ss, ss + want))
        manifest.append(dict(beat=i, kind='beat', rule=spec['rule'], cam=cam,
                             label=label, pinned=bool(pin),
                             source=os.path.basename(srcdir.rstrip('/')),
                             fell_back=fell_back,
                             ss=round(ss, 1), seconds=want, why=why,
                             stop_intervals=[(round(a, 2), round(b, 2))
                                             for a, b in stops]))
        print(f'beat {i}: {label:22s} {cam:6s} +{ss:6.1f}s  {why}')

    if not parts:
        print('nothing to assemble')
        return 1
    os.makedirs(os.path.dirname(os.path.abspath(args.dest)), exist_ok=True)
    if not concat(parts, args.dest):
        return 1
    with open(os.path.splitext(args.dest)[0] + '.cut.json', 'w') as fh:
        json.dump(manifest, fh, indent=2)
    dur = subprocess.run(['ffprobe', '-v', 'error', '-show_entries',
                          'format=duration', '-of', 'csv=p=0', args.dest],
                         capture_output=True, text=True).stdout.strip()
    print(f'\n{args.dest}  {float(dur):.1f} s  '
          f'{os.path.getsize(args.dest) / 1048576:.2f} MB')
    return 0


if __name__ == '__main__':
    sys.exit(main())
