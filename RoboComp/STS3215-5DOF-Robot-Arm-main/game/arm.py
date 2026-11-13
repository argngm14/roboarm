"""
arm.py - pose library + motion building blocks.

Poses are stored once, by name, in poses.json (raw servo ticks, 0-4095).
Only these names can be saved:

    safe            the safe position every route passes through
    b1_a ... b6_a   block aim   (hover above block N)
    b1_g ... b6_g   block grab  (gripper around block N)
    c1_a ... c9_a   cell aim    (hover in front of cell N)
    c1_p ... c9_p   cell place  (block inside cell N)
    c1_k ... c9_k   cell knock  (attack: pushes deeper into the cell than c_p;
                                 until recorded, attacks use c_p instead)

    Optional, per block type (1 trophy, 2 cube, 3 cylinder):
    c5_a1, c5_p1    cell 5 aim/place used ONLY when holding a trophy.
                    Other types keep using c5_a / c5_p. Same for a2/p2, a3/p3.

Each arm pose is 4 numbers: base, shoulder, elbow, wrist. The gripper is NOT
part of a pose; it's saved separately:
    o1 o2 o3 = open for trophy / cube / cylinder
    c1 c2 c3 = closed (squeeze) for trophy / cube / cylinder

Every route goes through "safe", so any block can go to any cell
without recording that combination.

Cell numbering (from YOUR robot's side of the rack):
    1 2 3
    4 5 6
    7 8 9

During a match, use game.py (it also tracks which blocks are used).
Single test move at game speed:    python game/arm.py 2 5
Same move at test speed:           python game/arm.py 2 5 test
"""

import json
import os
import time

import hardware as hw

# Block type numbers, used in the grip names o1-o3 / c1-c3.
TYPE_NUMBER = {"trophy": 1, "cube": 2, "cylinder": 3}

# Points per block type (rulebook 3.5.1.3). Patterns in game.py place high to low.
TYPE_POINTS = {"trophy": 40, "cylinder": 20, "cube": 10}

# Which block number is which type. Change to match how you lay them out.
#   1-2: trophies (left plate)   3-4: cubes (front plate)   5-6: cylinders (right plate)
BLOCK_TYPES = {
    1: "trophy", 2: "trophy",
    3: "cube", 4: "cube",
    5: "cylinder", 6: "cylinder",
}
CELLS = range(1, 10)

# Two speed settings. Speeds use the same units as arm_settings.py
# (course default: speed 3000, acceleration 254). Higher = faster.
#   travel: big moves (safe <-> aim)      fine: short moves (aim <-> grab/place)
#   accel:  higher = snappier starts/stops, lower = gentler
#   grip_wait: minimum seconds for the gripper to open/close; it also waits
#              until the gripper has actually stopped moving (lower = faster)
# Corner cutting ("blend"). The big moves (safe <-> aim) flow through "safe" without
# stopping: the next pose is sent as soon as every joint is within this many ticks
# of safe (150 ticks is about 13 degrees). Bigger = smoother and faster but cuts
# corners more; 0 = never blend.
# Aim <-> grab/place/knock is never blended: the arm stops fully at the aim pose,
# waits AIM_PAUSE seconds, then goes in, so grabbing and placing stay controlled.
# Only game.py turns blending on, and only at game speed. teach.py, arm.py and
# test speed stop at every pose with no pause, as before.
BLEND_TICKS = 150
AIM_PAUSE = 0.2

# sweeptest stops if the claw has moved more than this many ticks from where it
# started (a block that slips or drops lets the claw close further). A guess: tune it.
SWEEP_SLIP_TICKS = 80

SPEEDS = {
    "test": {"travel": 700, "fine": 180, "accel": 20, "grip_wait": 0.7, "blend": 0},   # teach mode uses this
    # travel 3000 = the course's default and about the servo's top speed at 12 V (~3070).
    # accel 254 is the course default (the most aggressive); 150 is a bit gentler.
    "game": {"travel": 3000, "fine": 1500, "accel": 150, "grip_wait": 0.5, "blend": BLEND_TICKS},
}
GRIP_SPEED = 1500

# Timing printout for finding where the pauses are. Off normally; turn on with:
#   ROBOARM_TRACE=1 python3 game.py
TRACE = os.environ.get("ROBOARM_TRACE") == "1"

# During an attack the empty gripper closes to this saved grip first, so it goes
# into the cell the same width as when it carried that block in. c2 = cube.
ATTACK_GRIP = "c2"

# Practice runs save to a separate file, so made-up numbers never reach the real arm.
_POSE_NAME = "poses_dryrun.json" if hw.DRY_RUN else "poses.json"
POSE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), _POSE_NAME)

# Which blocks have been used this game (only game.py tracks this).
_STATE_NAME = "game_state_dryrun.json" if hw.DRY_RUN else "game_state.json"
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), _STATE_NAME)


# ---------------------------------------------------------------- file I/O

def load_poses():
    if not os.path.exists(POSE_FILE):
        return {"arm": {}, "grip": {}}
    with open(POSE_FILE) as f:
        poses = json.load(f)
    poses.setdefault("arm", {})
    poses.setdefault("grip", {})
    return poses


def save_poses(poses):
    """One pose per line so the file stays readable and hand-editable."""
    sections = []
    for sec in ("arm", "grip"):
        items = sorted(poses.get(sec, {}).items())
        if items:
            body = ",\n".join(f"    {json.dumps(k)}: {json.dumps(v)}" for k, v in items)
            sections.append(f'  "{sec}": {{\n{body}\n  }}')
        else:
            sections.append(f'  "{sec}": {{}}')
    text = "{\n" + ",\n".join(sections) + "\n}\n"
    tmp = POSE_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, POSE_FILE)  # never leaves a half-written file


def required_poses():
    names = ["safe"]
    for b in BLOCK_TYPES:
        names += [f"b{b}_a", f"b{b}_g"]
    for c in CELLS:
        names += [f"c{c}_a", f"c{c}_p", f"c{c}_k"]
    return names


def valid_pose_names():
    """Every arm pose name that may be saved: the required ones plus per-type cell poses."""
    names = required_poses()
    for c in CELLS:
        for t in sorted(TYPE_NUMBER.values()):
            names += [f"c{c}_a{t}", f"c{c}_p{t}"]
    return names


POSE_NAME_HELP = (
    "pose names: safe | b1_a-b6_a (block aim) | b1_g-b6_g (block grab) | "
    "c1_a-c9_a (cell aim) | c1_p-c9_p (cell place) | c1_k-c9_k (cell knock, for attacks) | "
    "optional: c5_a1 / c5_p1 = cell 5 for a trophy only (1 trophy, 2 cube, 3 cylinder)"
)


class BlockTracker:
    """Remembers which blocks were picked this game. Saved to a file, so a restart keeps it."""

    def __init__(self):
        self.used = set()
        if os.path.exists(STATE_FILE):
            with open(STATE_FILE) as f:
                self.used = set(json.load(f).get("used", []))

    def _save(self):
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"used": sorted(self.used)}, f)
        os.replace(tmp, STATE_FILE)

    def is_used(self, block):
        return block in self.used

    def mark(self, block):
        self.used.add(block)
        self._save()

    def unmark(self, block):
        self.used.discard(block)
        self._save()

    def reset(self):
        self.used = set()
        self._save()

    def available(self):
        return [b for b in BLOCK_TYPES if b not in self.used]


def grip_names():
    """All valid grip names: o1 o2 o3 c1 c2 c3."""
    nums = sorted(TYPE_NUMBER.values())
    return [f"o{n}" for n in nums] + [f"c{n}" for n in nums]


def required_grips():
    nums = sorted({TYPE_NUMBER[t] for t in BLOCK_TYPES.values()})
    return [f"o{n}" for n in nums] + [f"c{n}" for n in nums]


def synced_profile(start, target, speed, accel):
    """
    Give each joint its own speed and acceleration, scaled by how far it has
    to travel, so all joints start and finish together. The path between
    two poses is then the same every time.
    """
    dist = {n: abs(target[n] - start[n]) for n in target}
    longest = max(dist.values()) or 1
    speeds = {n: max(1, round(speed * d / longest)) for n, d in dist.items()}
    accs = {n: max(1, round(accel * d / longest)) for n, d in dist.items()}
    return speeds, accs, longest


# ---------------------------------------------------------------- the arm

class Arm:
    def __init__(self, speed="test", track=False, blend=False):
        self.poses = load_poses()
        self.set_speed(speed)
        self.track = track                       # True in game.py: refuse to pick a used block
        self.allow_blend = blend                 # True in game.py only (see BLEND_TICKS)
        self.tracker = BlockTracker() if track else None
        self.bus = hw.open_bus()
        # Hold wherever the arm is right now. Nothing moves on startup.
        pos = self.bus.hold()
        self.current = [pos[n] for n in hw.ARM_JOINTS]
        self.grip_pos = pos[hw.GRIPPER]
        self.free = False        # True while arm torque is off for moving by hand
        self.grip_free = False   # True while gripper torque is off (claw moves by hand)
        self.holding = None      # block type in the gripper, or None
        self._in_flight = False  # True right after a blended move (arm may still be moving)

    def set_speed(self, name):
        if name not in SPEEDS:
            raise KeyError(f"Speed must be one of: {', '.join(SPEEDS)}")
        self.speed_name = name
        self.sp = SPEEDS[name]

    def close(self):
        self.bus.close()

    # --- torque on/off for teaching by hand ---

    def release(self):
        """
        Arm torque off (base, shoulder, elbow, wrist). SUPPORT THE ARM FIRST.
        The gripper stays on, so a block you're holding doesn't drop.
        """
        self.bus.torque(hw.ARM_JOINTS, False)
        self.free = True

    def hold(self):
        """Arm torque back on, holding wherever the arm is now. Gripper untouched."""
        pos = self.bus.hold(hw.ARM_JOINTS)
        self.current = [pos[n] for n in hw.ARM_JOINTS]
        self.free = False

    def release_gripper(self):
        """
        Gripper torque off, so the claw can be moved by hand (to fit it around a
        block without any squeeze). A block in the gripper will DROP.
        """
        self.bus.torque([hw.GRIPPER], False)
        self.grip_free = True

    def hold_gripper(self):
        """Gripper torque back on, holding wherever the claw is now (no jump)."""
        pos = self.bus.hold([hw.GRIPPER])
        self.grip_pos = pos[hw.GRIPPER]
        self.grip_free = False

    def refresh(self):
        """While free, read where the arm / gripper have been moved to by hand."""
        if self.free:
            pos = self.bus.read(hw.ARM_JOINTS)
            self.current = [pos[n] for n in hw.ARM_JOINTS]
        if self.grip_free:
            self.grip_pos = self.bus.read([hw.GRIPPER])[hw.GRIPPER]

    # --- low level ---

    def move(self, target, speed=None, accel=None, blend=0):
        """
        Move all 4 arm joints together and wait until they arrive.
        blend > 0: don't wait for a full stop; return once every joint is within
        `blend` ticks, so the next move() carries straight on (one continuous motion).
        """
        speed = speed or self.sp["travel"]
        accel = accel or self.sp["accel"]
        target = [int(round(x)) for x in target]
        goal = dict(zip(hw.ARM_JOINTS, target))
        bad = hw.outside_limits(goal)
        if bad:
            raise ValueError("Outside joint limits, nothing moved: " + hw.describe(bad))
        if self.free:
            self.hold()
        if self._in_flight:
            # The last move was blended, so the arm isn't at its target yet: plan from where it is.
            start = self.bus.read(hw.ARM_JOINTS)
        else:
            start = dict(zip(hw.ARM_JOINTS, self.current))
        speeds, accs, longest = synced_profile(start, goal, speed, accel)
        t0 = time.monotonic()
        self.bus.send(goal, speeds, accs)
        timeout = 3 + 2 * longest / speed
        try:
            if blend:
                self.bus.wait_near(hw.ARM_JOINTS, goal, blend, timeout)
            else:
                self.bus.wait(hw.ARM_JOINTS, timeout=timeout)
        except (hw.StopPressed, RuntimeError):
            # Stopped partway: remember where the arm actually is.
            pos = self.bus.read(hw.ARM_JOINTS)
            self.current = [pos[n] for n in hw.ARM_JOINTS]
            self._in_flight = False
            raise
        self.current = target
        self._in_flight = bool(blend)
        if TRACE:
            print(f"      {time.monotonic() - t0:.2f}s  {'blend' if blend else 'full stop'}"
                  f"  ({longest} ticks, speed {speed}, accel {accel})")

    def set_gripper(self, position, wait=True):
        """
        Move the gripper, then wait until it has stopped moving (at least
        grip_wait seconds). When it squeezes a block it stops before reaching
        its target, so this watches its position rather than a 'finished' signal.
        """
        position = int(round(position))
        bad = hw.outside_limits({hw.GRIPPER: position})
        if bad:
            raise ValueError("Outside gripper limit, nothing moved: " + hw.describe(bad))
        if self.grip_free:
            self.hold_gripper()   # torque back on where the claw is, then move: no jump
        t0 = time.monotonic()
        self.bus.send({hw.GRIPPER: position}, {hw.GRIPPER: GRIP_SPEED}, {hw.GRIPPER: self.sp["accel"]})
        self.grip_pos = position
        if wait:
            self.wait_gripper()
            if TRACE:
                print(f"      {time.monotonic() - t0:.2f}s  gripper wait")

    def wait_gripper(self, min_time=None):
        self.bus.wait_settled(hw.GRIPPER, self.sp["grip_wait"] if min_time is None else min_time)

    # --- named poses ---

    def blending(self):
        """True only in game.py at game speed."""
        return bool(self.allow_blend and self.sp["blend"])

    def go(self, name, kind="travel", blend=False):
        """blend=True: pass through this pose without stopping, if blending is on (see BLEND_TICKS)."""
        if name not in self.poses["arm"]:
            raise KeyError(f"No arm pose called '{name}' saved yet")
        print(f"  -> {name}")
        self.move(self.poses["arm"][name], self.sp[kind],
                  blend=self.sp["blend"] if blend and self.blending() else 0)

    def aim_pause(self):
        """Short still moment at the aim pose before going in to grab/place (blending only)."""
        if self.blending():
            time.sleep(AIM_PAUSE)

    def grip(self, name):
        if name not in self.poses["grip"]:
            raise KeyError(f"No gripper value called '{name}' saved yet")
        print(f"  gripper {name}")
        self.set_gripper(self.poses["grip"][name])
        # Keep track of what's in the gripper: c1 = holding a trophy, o1 = empty.
        if name[0] == "c":
            self.holding = self.type_from_number(int(name[1:]))
        else:
            self.holding = None

    # --- picking the right pose for the block ---

    @staticmethod
    def type_from_number(n):
        for t, num in TYPE_NUMBER.items():
            if num == n:
                return t
        raise KeyError(f"No block type {n}")

    def block_type(self, block):
        if block not in BLOCK_TYPES:
            raise KeyError(f"Block {block} isn't in BLOCK_TYPES (arm.py)")
        return BLOCK_TYPES[block]

    def cell_pose(self, cell, kind, btype):
        """kind 'a' or 'p'. Holding a trophy: 'c5_p1' if it was saved, otherwise 'c5_p'."""
        if cell not in CELLS:
            raise KeyError(f"Cell must be 1-9, not {cell}")
        special = f"c{cell}_{kind}{TYPE_NUMBER[btype]}" if btype else None
        return special if special in self.poses["arm"] else f"c{cell}_{kind}"

    def check(self, names, grips=()):
        """Fail BEFORE moving, not halfway through with a block in the gripper."""
        missing = [n for n in names if n not in self.poses["arm"]]
        missing += [f"grip:{g}" for g in grips if g not in self.poses["grip"]]
        if missing:
            raise KeyError("Missing poses: " + ", ".join(missing))
        problems = []
        for n in names:
            bad = hw.outside_limits(dict(zip(hw.ARM_JOINTS, self.poses["arm"][n])))
            if bad:
                problems.append(f"{n}: {hw.describe(bad)}")
        for g in grips:
            bad = hw.outside_limits({hw.GRIPPER: self.poses["grip"][g]})
            if bad:
                problems.append(f"grip {g}: {hw.describe(bad)}")
        if problems:
            raise ValueError("Saved poses outside joint limits, nothing moved: " + "; ".join(problems))

    # --- the actual task ---

    def pick(self, block, stop_at_end=True):
        """stop_at_end=False: leave the arm still moving into 'safe' (pick_and_place carries on)."""
        b = f"b{block}"
        n = TYPE_NUMBER[self.block_type(block)]
        self.check_unused(block)
        self.check(["safe", f"{b}_a", f"{b}_g"], [f"o{n}", f"c{n}"])
        self.grip(f"o{n}")
        self.go(f"{b}_a", "travel")                    # full stop at aim, then a short pause
        self.aim_pause()
        self.go(f"{b}_g", "fine")                      # full stop: the gripper closes here
        self.grip(f"c{n}")
        if self.track:
            self.tracker.mark(block)   # it's off its plate now
        self.go(f"{b}_a", "fine")                      # full stop: straight up out of the plate
        self.go("safe", "travel", blend=not stop_at_end)

    def check_unused(self, block):
        if self.track and self.tracker.is_used(block):
            raise KeyError(f"Block {block} was already used this game. "
                           f"'unuse {block}' if it's back on its plate, or 'reset' for a new game.")

    def place(self, cell):
        if not self.holding:
            raise KeyError("Not holding a block. Use 'pick B' first (or 'grip c1' etc.)")
        n = TYPE_NUMBER[self.holding]
        aim = self.cell_pose(cell, "a", self.holding)
        put = self.cell_pose(cell, "p", self.holding)
        self.check(["safe", aim, put], [f"o{n}"])
        self.go(aim, "travel")                         # full stop at aim, then a short pause
        self.aim_pause()
        self.go(put, "fine")                           # full stop: the gripper opens here
        self.grip(f"o{n}")
        self.go(aim, "fine")                           # full stop: straight out of the cell
        self.go("safe", "travel")

    def sweep(self, cells=None, at_place=None):
        """
        Fit check: with a block already in the gripper, take it to the place pose
        of every cell (or just the cells given), in and back out, WITHOUT releasing it.
        Route per cell is the same as place(): safe -> aim -> place -> aim -> safe.
        at_place(cell), if given, is called while the block is at the place pose;
        returning False stops the sweep after backing out.
        Stops early if the claw moves (block slipped or dropped).
        Returns the list of cells completed.
        """
        if not self.holding:
            raise KeyError("Not holding a block. Use 'pick B' first (or 'grip c1' etc.)")
        cells = list(cells) if cells else list(CELLS)
        bad = [c for c in cells if c not in CELLS]
        if bad:
            raise KeyError(f"Cells must be 1-9, not {bad}")
        route = {c: (self.cell_pose(c, "a", self.holding), self.cell_pose(c, "p", self.holding))
                 for c in cells}
        self.check(["safe"] + [p for pair in route.values() for p in pair])   # before anything moves
        print(f"sweep: holding a {self.holding}, cells {cells}, never releasing   "
              f"[{self.speed_name} speed, Space = stop]")
        grip_start = self.bus.read([hw.GRIPPER])[hw.GRIPPER]
        self.go("safe", "travel")
        done = []
        for c in cells:
            aim, put = route[c]
            print(f"cell {c}")
            self.go(aim, "travel")
            self.aim_pause()
            self.go(put, "fine")
            keep_going = at_place(c) if at_place else True
            self.go(aim, "fine")
            self.go("safe", "travel")
            done.append(c)
            moved = abs(self.bus.read([hw.GRIPPER])[hw.GRIPPER] - grip_start)
            if moved > SWEEP_SLIP_TICKS:
                print(f"  ! the gripper moved {moved} ticks, so the block may have slipped or dropped. "
                      f"Sweep stopped after cell {c}.")
                break
            if keep_going is False:
                print(f"  sweep stopped after cell {c}")
                break
        else:
            print(f"sweep done: {self.holding} reached all of {done} and is still held")
        return done

    def check_route(self, block, cell):
        """Everything pick_and_place(block, cell) needs, checked without moving."""
        btype = self.block_type(block)
        n = TYPE_NUMBER[btype]
        self.check_unused(block)
        self.check(
            ["safe", f"b{block}_a", f"b{block}_g",
             self.cell_pose(cell, "a", btype), self.cell_pose(cell, "p", btype)],
            [f"o{n}", f"c{n}"],
        )

    def pick_and_place(self, block, cell):
        btype = self.block_type(block)
        self.check_route(block, cell)
        print(f"block {block} ({btype}) -> cell {cell}   [{self.speed_name} speed, Space = stop]")
        self.go("safe", "travel", blend=True)
        self.pick(block, stop_at_end=False)   # flows straight into place: no stop at safe
        self.place(cell)

    def attack(self, cell):
        """
        Knock the block out of a cell: close the empty gripper, then the same
        route as placing (aim -> in -> aim -> safe) with no release. Goes to
        c_k if saved (deeper push), otherwise c_p. Ends at safe, which must be
        outside the Tic-Tac-Toe Zone so the attack attempt ends.
        """
        if self.holding:
            raise KeyError(f"Holding a {self.holding}. Place it first, then attack.")
        if cell not in CELLS:
            raise KeyError(f"Cell must be 1-9, not {cell}")
        aim = f"c{cell}_a"
        knock = f"c{cell}_k" if f"c{cell}_k" in self.poses["arm"] else f"c{cell}_p"
        self.check(["safe", aim, knock], [ATTACK_GRIP])
        print(f"attack cell {cell} via {knock}   [{self.speed_name} speed, Space = stop]")
        self.go("safe", "travel")
        self.grip(ATTACK_GRIP)
        self.holding = None            # closed, but nothing in it
        self.go(aim, "travel")                         # full stop at aim, then a short pause
        self.aim_pause()
        self.go(knock, "fine")                         # full stop: reach the full knock depth
        self.go(aim, "fine")
        self.go("safe", "travel")


if __name__ == "__main__":
    import sys
    if len(sys.argv) not in (3, 4):
        print("usage: python game/arm.py BLOCK CELL [test]")
        sys.exit(1)
    speed = "test" if len(sys.argv) == 4 and sys.argv[3] == "test" else "game"
    arm = Arm(speed)
    try:
        arm.pick_and_place(int(sys.argv[1]), int(sys.argv[2]))
    except hw.StopPressed:
        print("\nStopped. Arm is holding still.")
    finally:
        arm.close()
