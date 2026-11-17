import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "game"))

import motion  # noqa: E402

NAMES = ["base_yaw", "shoulder", "elbow", "wrist"]
# What each simulated motor can really do (steps/s^2, steps/s). The shoulder is the weak, loaded one.
WEAK_SHOULDER = {"base_yaw": (15000, 3000), "shoulder": (6000, 2200),
                 "elbow": (12000, 3000), "wrist": (15000, 3000)}


class SimArm:
    """Servos that drive to their latest target, limited by the command AND by what the motor can do."""

    def __init__(self, start, capability, read_cost=0.0):
        self.t = 0.0
        self.read_cost = read_cost          # seconds each servo read takes (the real Pi is not instant)
        self.cap = capability
        self.x = {n: float(start[n]) for n in NAMES}
        self.v = {n: 0.0 for n in NAMES}
        self.tgt = dict(self.x)
        self.vcap = {n: 3000 for n in NAMES}
        self.acc = {n: 254 for n in NAMES}
        self.log = []
        self.sent = []

    def send(self, targets, speeds, accs):
        self.sent.append(dict(targets))
        for n in targets:
            self.tgt[n], self.vcap[n], self.acc[n] = targets[n], speeds[n], accs[n]

    def read(self, names=NAMES):
        if self.read_cost:
            self.step(self.read_cost * len(names))
        return {n: round(self.x[n]) for n in NAMES}

    def step(self, dt):
        h = 0.001
        for _ in range(int(round(dt / h))):
            for n in NAMES:
                a = min(self.acc[n] * 100, self.cap[n][0])
                vmax = min(self.vcap[n], self.cap[n][1])
                e = self.tgt[n] - self.x[n]
                v_want = math.copysign(min(vmax, math.sqrt(2 * a * abs(e))), e)
                self.v[n] += max(-a * h, min(a * h, v_want - self.v[n]))
                self.x[n] += self.v[n] * h
            self.t += h
            self.log.append(dict(self.x))

    def settle(self, goal):
        while self.t < 10 and max(abs(goal[n] - self.x[n]) for n in NAMES) > 2:
            self.step(0.001)


def drift(log, start, goal):
    """Largest gap between the joints' progress along the move. 0 = perfectly straight in joint space."""
    delta = {n: goal[n] - start[n] for n in NAMES}
    longest = max(abs(d) for d in delta.values())
    big = [n for n in NAMES if abs(delta[n]) >= max(0.2 * longest, 30)]
    worst = 0.0
    for x in log:
        s = [min(max((x[n] - start[n]) / delta[n], 0), 1) for n in big]
        worst = max(worst, max(s) - min(s))
    return worst


# shoulder goes down while the elbow goes up, so their vertical motion cancels (a straight horizontal path)
START = {"base_yaw": 2048, "shoulder": 2200, "elbow": 1900, "wrist": 2048}
GOAL = {"base_yaw": 2048, "shoulder": 1700, "elbow": 2550, "wrist": 2150}


def classic_move(sim, speed, accel_units):
    """The old way: give each joint its own speed/accel scaled by distance and let the servos run."""
    longest = max(abs(GOAL[n] - START[n]) for n in NAMES)
    speeds = {n: max(1, round(speed * abs(GOAL[n] - START[n]) / longest)) for n in NAMES}
    accs = {n: max(1, round(accel_units * abs(GOAL[n] - START[n]) / longest)) for n in NAMES}
    sim.send(GOAL, speeds, accs)
    sim.settle(GOAL)


def coupled_move(sim, speed, accel_units, stop=lambda: False, timeout=10):
    sim.read_cost = sim.read_cost or 0.02          # assume a slow 20 ms per servo read
    result = motion.run_coupled(sim.read, sim.send, stop, NAMES, START, GOAL, speed, accel_units, timeout,
                                clock=lambda: sim.t, sleep=sim.step)
    sim.settle(GOAL)
    return result


class ProfileTests(unittest.TestCase):
    def test_trapezoid_and_triangle_reach_the_distance(self):
        for distance in (1500, 100):
            total, x = motion.profile(distance, 3000, 15000)
            self.assertEqual(x(0), 0)
            self.assertEqual(x(total), distance)
            samples = [x(total * i / 50) for i in range(51)]
            self.assertEqual(samples, sorted(samples))

    def test_zero_distance(self):
        total, x = motion.profile(0, 3000, 15000)
        self.assertEqual((total, x(1)), (0.0, 0.0))


class PathTests(unittest.TestCase):
    def test_slow_classic_move_is_straight(self):
        sim = SimArm(START, WEAK_SHOULDER)
        classic_move(sim, 700, 2000)
        self.assertLess(drift(sim.log, START, GOAL), 0.05)

    def test_fast_classic_move_bends(self):
        # the problem being fixed: the weak shoulder falls behind, so the path curves
        sim = SimArm(START, WEAK_SHOULDER)
        classic_move(sim, 3000, 15000)
        self.assertGreater(drift(sim.log, START, GOAL), 0.15)

    def test_coupled_fast_move_stays_straight_and_arrives(self):
        sim = SimArm(START, WEAK_SHOULDER)
        self.assertEqual(coupled_move(sim, 3000, 150), "done")
        self.assertLess(drift(sim.log, START, GOAL), 0.08)       # the old way bends > 15%
        self.assertEqual(sim.sent[-1], GOAL)            # the last target is exactly the goal
        for n in NAMES:
            self.assertAlmostEqual(sim.x[n], GOAL[n], delta=3)

    def test_coupled_move_is_not_throttled_by_slow_reads(self):
        # Regression: an earlier version capped the speed at (lead / read time), so on the
        # real Pi every game-speed move crawled. With motors that keep up and slow reads,
        # a coupled move must take about as long as a plain one.
        strong = {n: (15000, 3000) for n in NAMES}
        plain = SimArm(START, strong)
        classic_move(plain, 3000, 15000)
        sim = SimArm(START, strong, read_cost=0.02)
        coupled_move(sim, 3000, 150)
        self.assertLess(sim.t, 1.6 * plain.t)


class SafetyTests(unittest.TestCase):
    def test_stop_returns_stop_and_sends_nothing_more(self):
        sim = SimArm(START, WEAK_SHOULDER)
        calls = {"n": 0}

        def stop():
            calls["n"] += 1
            return calls["n"] > 5

        self.assertEqual(motion.run_coupled(sim.read, sim.send, stop, NAMES, START, GOAL, 3000, 150, 10,
                                            clock=lambda: sim.t, sleep=sim.step), "stop")
        self.assertLessEqual(len(sim.sent), 5)

    def test_a_blocked_joint_times_out_instead_of_pushing(self):
        capability = dict(WEAK_SHOULDER, shoulder=(1, 1))          # shoulder can't move at all
        sim = SimArm(START, capability)
        result = motion.run_coupled(sim.read, sim.send, lambda: False, NAMES, START, GOAL, 3000, 150, 2.0,
                                    clock=lambda: sim.t, sleep=sim.step)
        self.assertEqual(result, "timeout")
        # nothing ran far ahead of the stuck joint
        furthest = max(abs(sim.tgt["elbow"] - START["elbow"]), 1)
        self.assertLess(furthest, 0.25 * abs(GOAL["elbow"] - START["elbow"]))

    def test_no_distance_does_nothing(self):
        sim = SimArm(START, WEAK_SHOULDER)
        self.assertEqual(motion.run_coupled(sim.read, sim.send, lambda: False, NAMES, START, START, 3000, 150, 5,
                                            clock=lambda: sim.t, sleep=sim.step), "done")
        self.assertEqual(sim.sent, [])

    def test_one_bad_read_is_skipped_but_five_in_a_row_raise(self):
        sim = SimArm(START, WEAK_SHOULDER)
        fails = {"left": 1}

        def flaky(names):
            if fails["left"] > 0:
                fails["left"] -= 1
                raise RuntimeError("garbled")
            return sim.read()

        self.assertEqual(motion.run_coupled(flaky, sim.send, lambda: False, NAMES, START, GOAL, 3000, 150, 10,
                                            clock=lambda: sim.t, sleep=sim.step), "done")

        def dead(names):
            raise RuntimeError("no reply")

        with self.assertRaises(RuntimeError):
            motion.run_coupled(dead, sim.send, lambda: False, NAMES, START, GOAL, 3000, 150, 10,
                               clock=lambda: sim.t, sleep=sim.step)


if __name__ == "__main__":
    unittest.main()
