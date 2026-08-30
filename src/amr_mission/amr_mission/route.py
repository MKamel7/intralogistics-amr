#!/usr/bin/env python3
"""Where the load is picked up and where it is set down.

This is the handling rule for a transport route, kept apart from the node that
executes it so it can be tested without a ROS graph, a simulator or a running
Nav2 stack. The rule is four lines; the bug it replaced was invisible for
exactly as long as nothing tested it.

WHAT WAS WRONG. The task used to decide the load state with `carrying =
(leg == 0)`, which reads as "laden after the first stop, unladen after every
other one". On the two-stop route that ships in `config/stations.yaml` that is
correct and indistinguishable from the rule below: load at `goods_in`, unload
at `dispatch`. On any longer route it is wrong. A three-stop route set the box
down at the middle station and drove the final leg empty, while the cycle was
still recorded as a completed laden delivery, so every laden-leg number in
`docs/validation.md` would have been measured on a vehicle that was not
carrying anything. The comment in `stations.yaml` promising that "a three-stop
or a loop route needs no code change" was the claim this broke.

That is the failure worth naming: not a crash, but a wrong answer that still
reports success.
"""


def carrying_after(leg, stops):
    """Is the vehicle laden once it has finished handling at stop `leg`?

    The first stop loads, the last stop unloads, and a stop in between is a
    transfer that the load rides straight through. So the vehicle is carrying
    after every stop except the final one.

    `leg` is zero-based. `stops` is `len(route)`.
    """
    if stops < 2:
        raise ValueError(
            f'a transport route needs at least two stops, got {stops}')
    if not 0 <= leg < stops:
        raise ValueError(f'leg {leg} is not a stop on a {stops}-stop route')
    return leg != stops - 1


def handling_label(leg, stops):
    """What is happening at this stop, for the state topic and the log.

    Kept beside `carrying_after` so the words and the behaviour cannot drift
    apart: a run that says "unloading" at a stop where the load stays on the
    plate is a report that disagrees with the vehicle.
    """
    if stops < 2:
        raise ValueError(
            f'a transport route needs at least two stops, got {stops}')
    if not 0 <= leg < stops:
        raise ValueError(f'leg {leg} is not a stop on a {stops}-stop route')
    if leg == 0:
        return 'loading'
    if leg == stops - 1:
        return 'unloading'
    return 'passing through, still laden'
