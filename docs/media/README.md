# Demo video

`demo.mp4`, 30 s, 1280x720, no audio.

    0:00  driving a rack aisle during the survey, walls sweeping past
    0:08  passing pedestrians at close quarters in the open bay
    0:17  arriving at the dispatch bay carrying a 100 kg box, with an earlier
          delivery already on the table
    0:24  RViz: the SLAM map, both costmaps inflated, and the MPPI candidate
          trajectory fan ahead of the vehicle

The first cut of this video was passive and it was cut badly. Its "transport
cycle" segment showed a vehicle that was almost stationary, because it came from
a cycle that FAILED to reach goods_in and was being held up by protective stops:
a stalled robot presented as a driving one. Its delivery segment started eight
seconds after the box had already been placed, so it was a static table.

Both came from choosing timestamps without looking at the frames. The fix was a
contact sheet: one command, thirty thumbnails, and both mistakes are obvious.
The footage here is from the SURVEY rather than the mission, because the survey
drives continuous twenty to fifty metre legs while the mission spends most of
its time stopped at stations.

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

**A complete run.** Cycle completion on this track varies from 0 to 3 and the
README carries the distribution rather than a best case.

**A load being placed.** `set_pose` moves the box instantaneously, so there is
no gradual placing motion in this model to film. What the video shows instead is
the vehicle arriving with a box on its deck and an earlier delivery sitting on
the table, which is the honest version of the same claim. A gripper or a lifting
deck would be a different project.
