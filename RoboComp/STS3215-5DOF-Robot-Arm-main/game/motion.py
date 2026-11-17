"""
motion.py - "coupled" moves that keep the arm on a straight line in joint space.

Why this exists
---------------
A normal move gives every joint its own speed and acceleration, scaled by how
far it has to go, so all joints should start and finish together. That works
while every motor can do what it is told. At high speed the heavier joints
(shoulder, elbow) cannot accelerate as fast as the lighter ones, so they fall
behind in the middle of the move and catch up at the end. The gripper then
follows a curve instead of a straight line, and the bend grows with speed.

A coupled move fixes that by feedback. It sends a new target many times a
second, always on the straight line from start to goal, following the normal
speed plan. It also watches how far apart the joints are (the "spread": how
much further along the move one joint is than another). All joints lagging
by the same amount is normal and harmless, so that does not slow anything.
Only when the spread grows past SPREAD_OK does the plan slow down, and it
drops to MIN_RATE at SPREAD_STOP, so the lagging joint catches up and the
path stays straight.

This file only holds the logic (no serial port), so it can be tested on a
laptop with a simulated arm. hardware.py wires it to the real motors.
"""

import math
import time

TICK = 0.02          # seconds between target updates
SPREAD_OK = 0.02     # joints may be up to 2% of the move apart before the plan slows down
SPREAD_STOP = 0.06   # at 6% apart the plan runs at only MIN_RATE until the lagging joint catches up.
MIN_RATE = 0.15      # never stop the plan completely: a joint that sags a little (gravity, a held
                     # block) never closes the gap, and a full pause would then wait forever
SPREAD_HALT = 0.20   # this far apart (after SAG_TICKS) a joint must be stuck: stop the plan and let
                     # the timeout stop the arm, instead of dragging the other joints on without it
SAG_TICKS = 15       # each joint may be this many ticks off the line without counting as spread
                     # (sag and servo wobble, which matter on short moves like the lift after a grab)
                     # Smaller = straighter but slower; bigger = faster but bends more.
MAX_SPEED = 3000     # servo speed cap sent with each target (same as the course default)
SERVO_ACCEL = 254    # servo's own acceleration setting: the highest, so it follows each target tightly
ACC_UNIT = 100       # one acceleration unit = 100 steps/s^2 (taken from the Feetech docs; unverified here)
MAX_BAD_READS = 5    # this many failed reads in a row = give up


def profile(distance, speed, accel):
    """
    Trapezoid for the longest joint: accelerate, cruise, brake (or just
    accelerate and brake if the move is too short to reach full speed).
    speed in ticks/s, accel in ticks/s^2. Returns (total_time, x) where x(t)
    is how far along the move (in ticks) the longest joint should be at time t.
    """
    if distance <= 0:
        return 0.0, lambda t: 0.0
    if distance >= speed * speed / accel:
        t_acc = speed / accel
        d_acc = speed * speed / (2 * accel)
        t_cruise = (distance - 2 * d_acc) / speed
        total = 2 * t_acc + t_cruise
    else:
        peak = math.sqrt(accel * distance)
        t_acc = peak / accel
        d_acc = 0.5 * accel * t_acc * t_acc
        t_cruise = 0.0
        total = 2 * t_acc

    def x(t):
        if t <= 0:
            return 0.0
        if t >= total:
            return float(distance)
        if t < t_acc:
            return 0.5 * accel * t * t
        if t < t_acc + t_cruise:
            return d_acc + speed * (t - t_acc)
        left = total - t
        return distance - 0.5 * accel * left * left

    return total, x


def run_coupled(read, send, stop, names, start, goal, speed, accel, timeout,
                clock=time.monotonic, sleep=time.sleep, tick=TICK,
                spread_ok=None, spread_stop=None):
    """
    Drive every joint in `names` from `start` to `goal` along the straight line
    between them. Returns "done" (the final target has been sent), "stop"
    (stop() returned True) or "timeout".

    read(joints)                 -> {name: position} for those joints
    send(targets, speeds, accs)  -> sends one synchronized target for every joint
    stop()                       -> True if the user asked to stop
    speed: ticks/s of the longest joint. accel: servo units (x100 steps/s^2).
    """
    spread_ok = SPREAD_OK if spread_ok is None else spread_ok
    spread_stop = SPREAD_STOP if spread_stop is None else spread_stop
    delta = {n: goal[n] - start[n] for n in names}
    longest = max(abs(d) for d in delta.values())
    if longest == 0:
        return "done"

    # Joints that move a meaningful amount are the ones whose spread bends the path.
    # (A joint that barely moves would only add noise.) Only these are read.
    big = [n for n in names if abs(delta[n]) >= max(0.2 * longest, 30)] or \
          [max(names, key=lambda n: abs(delta[n]))]

    # Speed cap for each joint, a bit above what it needs, so a joint that
    # moves little is not allowed to dash ahead.
    caps = {n: max(50, min(MAX_SPEED, int(1.5 * speed * abs(delta[n]) / longest) + 50)) for n in names}
    accs = {n: SERVO_ACCEL for n in names}

    _, x = profile(longest, max(1.0, speed), max(1.0, accel * ACC_UNIT))

    t0 = clock()
    last_t = t0
    plan_t = 0.0                    # time along the speed plan; runs slower than the clock while joints spread
    s_sent = 0.0                    # how far along the line the last target was (0..1)
    last = {n: start[n] for n in big}
    bad_reads = 0
    while True:
        if stop():
            return "stop"
        now = clock()
        if now - t0 > timeout:
            return "timeout"
        try:
            pos = read(big)
            bad_reads = 0
            last = pos
        except RuntimeError:
            bad_reads += 1
            if bad_reads >= MAX_BAD_READS:
                raise
            pos = last

        # How far behind its own last target is each joint, as a share of its move?
        # The first SAG_TICKS of that don't count.
        behind = []
        for n in big:
            ticks = (s_sent * delta[n] - (pos[n] - start[n])) * (1 if delta[n] > 0 else -1)
            ticks = max(0.0, abs(ticks) - SAG_TICKS) * (1 if ticks >= 0 else -1)
            behind.append(ticks / abs(delta[n]))
        spread = max(behind) - min(behind)
        if spread <= spread_ok:
            rate = 1.0
        elif spread >= SPREAD_HALT:
            rate = 0.0
        elif spread >= spread_stop:
            rate = MIN_RATE
        else:
            rate = 1.0 - (1.0 - MIN_RATE) * (spread - spread_ok) / (spread_stop - spread_ok)
        plan_t += (now - last_t) * rate
        last_t = now

        s_now = max(s_sent, min(x(plan_t) / longest, 1.0))
        send({n: round(start[n] + s_now * delta[n]) for n in names}, caps, accs)
        s_sent = s_now
        if s_now >= 1.0:
            return "done"
        sleep(max(0.0, tick - (clock() - now)))
