# Demo video

`demo.mp4`, 30 s, 1280x720, no audio.

    0:00  a transport cycle in progress, the vehicle sharing an aisle with people
    0:10  a delivered load set down on the table beside the dispatch station
    0:20  RViz: the SLAM map, both costmaps inflated, and the MPPI candidate
          trajectory fan ahead of the vehicle

## It is about 2.3 times real time, and that is not a stylistic choice

The recording is frame grabbed with `import -window`, because this machine runs
Wayland and `ffmpeg -f x11grab` captures the X root window, which an XWayland
client is never drawn into. Three clips were recorded that way first and every
frame of all three was solid black, at plausible file sizes and durations.

Grabbing a 1000x844 window costs about 130 ms, so the achievable rate is 4.3
frames per second against the 10 the file is encoded at. Measured, not assumed:
the first version of this note said "real speed" because 10 was the number
passed to ffmpeg.

The consequence for a viewer is that the vehicle moves faster than it does. The
protective stops are still visible but brief.

## What it does not show

**Docking.** It is built, unit tested and measured, and it does not work: 84
percent of its detections are not the dock, and its p95 error is above the
localisation floor it existed to beat. See V-66. The vehicle in this video parks
by navigation goal, which is what the README claims.

**A complete run.** The run this was recorded from completed 1 of 3 cycles.
Cycle completion on this track varies from 0 to 3 and the README carries the
distribution rather than a best case.
