"""
teach.py - record poses by name, either by moving the arm by hand or by
setting/nudging motors with typed commands.

On the Pi, from the STS3215-5DOF-Robot-Arm folder:
    source .venv/bin/activate
    python game/teach.py
"""

import json
import re
import sys
import time
from pathlib import Path

try:
    import readline  # noqa: F401  (up-arrow command history; not on Windows, that's fine)
except ImportError:
    pass

import arm as arm_lib
import hardware as hw

HELP = """
  TEACH BY HAND
  free             arm torque OFF so you can move it by hand  (SUPPORT THE ARM FIRST)
                   the gripper stays on, so a held block won't drop
  hold             arm torque ON, holds wherever it is now
  freegrip         GRIPPER torque OFF: move the claw by hand around a block, no squeeze
                   (a block in the gripper will drop). Then 'savegrip NAME' saves where it is.
  holdgrip         gripper torque ON, holds wherever it is now
                   (grip / set / nudge / pick turn it back on by themselves)

  SET / NUDGE MOTORS   order: base shoulder elbow wrist gripper  (raw ticks 0-4095)
  2000 - +50 -30 -     all 5 at once:  number = set to it,  +N / -N = nudge,  - = leave alone
  1 +10                one motor:      nudge motor 1 by +10
  2 1800               one motor:      set motor 2 to exactly 1800
                       (sets and nudges stop at the joint limits in hardware.py)

  SAVE                 (arm poses store motors 1-4 only; the gripper is saved separately)
  save NAME        save the arm position as NAME. Only these names are accepted:
                     safe
                     b1_a .. b6_a   block aim      b1_g .. b6_g   block grab
                     c1_a .. c9_a   cell aim       c1_p .. c9_p   cell place
                     c1_k .. c9_k   cell knock (attack push, deeper than c_p)
                     c5_a1 / c5_p1  optional: cell 5 used only for a trophy
                                    (1 trophy, 2 cube, 3 cylinder)
  savegrip NAME    save the gripper position:  o1 o2 o3 = open,  c1 c2 c3 = closed
                   (1 trophy, 2 cube, 3 cylinder)
  edit NAME ...    change a SAVED pose without moving the arm (no need to 'go' there first):
                     edit c3_p 1 +20        motor 1 of c3_p, nudge +20
                     edit c3_p - - -30 -    all 4 motors: '-' = leave alone, +N/-N = nudge, number = set
                     edit c2 +30            a saved gripper value
                   It prints old -> new and saves straight away. 'go NAME' to try it.

  TEST
  go NAME          move to a saved pose   (direct move, does NOT go via safe)
  grip NAME        set the gripper to a saved value (o1 ... c3)
  pick B           pick up block B and come back to safe, still holding it
                   (do this before recording cell poses, so a real block is in the gripper)
  place C          put the held block into cell C and come back to safe
  run B C          full pick-and-place: block B into cell C, via safe
  attack C         knock cell C: gripper closes, then c_a -> c_k (or c_p) -> c_a -> safe
                   (teach mode doesn't track used blocks, so you can repeat picks freely)
  speed test|game  which speed pick/place/run/go use  (set in arm.py; starts on test)

  INFO
  show             where the motors are right now
  watch            live readout while you move the arm by hand; press any key to stop
  list             all saved poses
  missing          poses you still need to record
  limits           the allowed range for each motor (JOINT_LIMITS in hardware.py)

  REUSE OLD WAYPOINT FILES (motions/ folder)
  points SEQ       list the points in motions/SEQ.json
  take SEQ N NAME  save point N of motions/SEQ.json as arm pose NAME  (e.g. take 1 3 b1_g)

  help / quit      (Space during any move = stop and hold)
"""

JOG_SPEED = 400
JOG_ACCEL = 20
BIG_JUMP = 800          # ask before a set/nudge moves an arm motor further than this (about 70 degrees)
MOTIONS = Path(__file__).resolve().parents[1] / "motions"
MOTOR_TOKEN = re.compile(r"^(-|[+-]?\d+)$")


def show(a):
    a.refresh()
    print(f"  target  {' '.join(str(v) for v in a.current)} {a.grip_pos}"
          "    (base shoulder elbow wrist gripper)")
    if not hw.DRY_RUN and not a.free:
        actual = a.bus.read()
        print(f"  actual  {' '.join(str(actual[n]) for n in hw.ALL_JOINTS)}")
    bad = hw.outside_limits(dict(zip(hw.ALL_JOINTS, list(a.current) + [a.grip_pos])))
    if bad:
        print("  ! outside joint limits: " + hw.describe(bad))
    notes = [f"{a.speed_name} speed"]
    if a.free:
        notes.append("arm torque OFF" + (", gripper torque OFF" if a.grip_free else ", gripper on"))
    elif a.grip_free:
        notes.append("GRIPPER torque OFF")
    if a.holding:
        notes.append(f"holding a {a.holding}")
    print("  (" + ", ".join(notes) + ")")


def watch(a):
    """Keep printing the real positions until a key is pressed."""
    keys = getattr(a.bus, "RawTerminal", None)
    if hw.DRY_RUN or keys is None or not sys.stdin.isatty():
        show(a)
        return
    print("  (base shoulder elbow wrist gripper)   press any key to stop")
    with keys() as term:
        while True:
            pos = a.bus.read()
            line = " ".join(f"{pos[n]:4d}" for n in hw.ALL_JOINTS)
            bad = hw.outside_limits(pos)
            flag = "   ! outside limit: " + ", ".join(n for n, *_ in bad) if bad else ""
            sys.stdout.write("\r  " + line + flag + " " * 20)
            sys.stdout.flush()
            if term.read_key(0.1):
                break
    print()
    a.refresh()


def move_motors(a, tokens):
    """tokens: 5 entries, one per motor. number = set, +N/-N = nudge, '-' = leave alone."""
    a.refresh()  # if the arm was moved by hand, work from where it is now
    now = list(a.current) + [a.grip_pos]
    target = list(now)
    for i, tok in enumerate(tokens):
        if tok == "-":
            continue
        value = int(tok)
        wanted = now[i] + value if tok[0] in "+-" else value
        if not 0 <= wanted <= 4095:
            print(f"  motor {i + 1} would go to {wanted}, outside 0-4095. Nothing moved.")
            return
        name = hw.ALL_JOINTS[i]
        target[i] = hw.clamp(name, wanted)
        if target[i] != wanted:
            lo, hi = hw.JOINT_LIMITS[name]
            print(f"  motor {i + 1} ({name}) stopped at its limit {target[i]} (limit {lo}-{hi}, asked for {wanted})")

    # arm motors only; the gripper going fully open/closed is a normal big move
    big = [(i + 1, now[i], target[i]) for i in range(4) if abs(target[i] - now[i]) > BIG_JUMP]
    if big:
        desc = ", ".join(f"motor {m}: {old} -> {new}" for m, old, new in big)
        if input(f"  Big move ({desc}). Type y to go: ").strip().lower() != "y":
            print("  cancelled")
            return

    arm_changes = target[:4] != now[:4]
    grip_changes = target[4] != now[4]
    started = time.monotonic()
    if grip_changes:
        a.set_gripper(target[4], wait=False)  # gripper moves at the same time as the arm
    if arm_changes:
        a.move(target[:4], JOG_SPEED, JOG_ACCEL)
    if grip_changes:
        # the arm may finish first; wait for the gripper to stop too
        a.wait_gripper(min_time=max(0, a.sp["grip_wait"] - (time.monotonic() - started)))
    show(a)


def edit_pose(a, tokens):
    """
    edit NAME v1 v2 v3 v4   change a saved arm pose (motors 1-4)
    edit NAME M value       change one motor of it
    edit NAME value         change a saved gripper value (o1..c3)
    Values: number = set to it, +N / -N = nudge, '-' = leave alone.
    Only the saved file changes; the arm does not move.
    """
    name, vals = tokens[0].lower(), tokens[1:]
    is_grip = name in arm_lib.grip_names()
    if not is_grip and name not in arm_lib.valid_pose_names():
        print(f"  '{name}' isn't a valid pose name. Nothing changed.")
        print("  " + arm_lib.POSE_NAME_HELP)
        return
    old = a.poses["grip" if is_grip else "arm"].get(name)
    if old is None:
        print(f"  '{name}' isn't saved yet, so there is nothing to edit. Use 'save' or 'savegrip' first.")
        return
    old = list(old) if not is_grip else [old]
    count = len(old)                       # 4 motors for an arm pose, 1 for a gripper value
    first_motor = 5 if is_grip else 1      # motor number of old[0]

    if len(vals) == 2 and vals[0].isdigit():          # "M value": one motor
        m = int(vals[0])
        if not first_motor <= m < first_motor + count or not MOTOR_TOKEN.match(vals[1]) or vals[1] == "-":
            print(f"  use e.g. 'edit {name} " + ("5 +20" if is_grip else "3 -25") + "'")
            return
        spec = ["-"] * count
        spec[m - first_motor] = vals[1]
    elif len(vals) == count and all(MOTOR_TOKEN.match(t) for t in vals):
        spec = vals
    else:
        print(f"  expected {count} value{'s' if count > 1 else ''} (or 'motor value') after the name. Nothing changed.")
        return

    new = list(old)
    for i, tok in enumerate(spec):
        if tok == "-":
            continue
        wanted = old[i] + int(tok) if tok[0] in "+-" else int(tok)
        if not 0 <= wanted <= 4095:
            print(f"  motor {first_motor + i} would be {wanted}, outside 0-4095. Nothing changed.")
            return
        joint = hw.ALL_JOINTS[first_motor - 1 + i]
        new[i] = hw.clamp(joint, wanted)
        if new[i] != wanted:
            lo, hi = hw.JOINT_LIMITS[joint]
            print(f"  motor {first_motor + i} ({joint}) stopped at its limit {new[i]} (limit {lo}-{hi}, asked for {wanted})")

    if is_grip:
        a.poses["grip"][name] = new[0]
    else:
        a.poses["arm"][name] = new
    arm_lib.save_poses(a.poses)
    shown = (lambda v: v[0]) if is_grip else (lambda v: v)
    print(f"  {name}: {shown(old)} -> {shown(new)}   (saved; the arm did not move)")
    print(f"  Try it with 'go {name}'" if not is_grip else f"  Try it with 'grip {name}'")


def warn_limits(targets):
    bad = hw.outside_limits(targets)
    if bad:
        print("  ! warning: outside joint limits, so this won't play until fixed: " + hw.describe(bad))


# --- reading the course's waypoint files ---

def load_sequence_points(seq_name):
    path = MOTIONS / f"{seq_name}.json"
    if not path.exists():
        raise KeyError(f"No file {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("type") != "pose_sequence":
        raise KeyError(f"{path.name} isn't a waypoint sequence")
    input_type = data.get("input_type")
    points = []
    for p in data.get("points", []):
        values = p.get("input_values")
        if values is None:  # older file format
            raw = p.get("positions_raw", {})
            points.append({n: int(raw[n]) for n in hw.ALL_JOINTS})
        elif input_type == "raw":
            points.append({n: int(v) for n, v in zip(hw.ALL_JOINTS, values)})
        elif input_type == "angles":
            # Angles need the course's calibration (arm_settings.py) to become ticks.
            repo = str(Path(__file__).resolve().parents[1])
            if repo not in sys.path:
                sys.path.insert(0, repo)
            from arm_helpers import angle_to_position
            points.append({n: angle_to_position(n, float(v)) for n, v in zip(hw.ALL_JOINTS, values)})
        else:
            raise KeyError(f"{path.name}: unknown input_type {input_type}")
    return points


def main():
    a = arm_lib.Arm(speed="test")

    if hw.DRY_RUN:
        print("=== PRACTICE MODE: motors are NOT moving. Saving to poses_dryrun.json ===")
        print("=== Set DRY_RUN = False in hardware.py to use the real arm.          ===")
    else:
        print("Connected. The arm is holding where it is; nothing has moved.")
        print("Space during a move = stop and hold. Keep the physical stop within reach.")
    show(a)
    print("Type 'help' for commands.")

    while True:
        try:
            line = input("teach> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        tokens = line.split()
        cmd = tokens[0].lower()

        try:
            if len(tokens) == 5 and all(MOTOR_TOKEN.match(t) for t in tokens):
                move_motors(a, tokens)

            elif cmd.isdigit() and len(tokens) == 2:
                motor = int(cmd)
                if not 1 <= motor <= 5 or not MOTOR_TOKEN.match(tokens[1]) or tokens[1] == "-":
                    print("  use e.g. '3 -25' or '2 1800' (motors 1-5)")
                    continue
                five = ["-"] * 5
                five[motor - 1] = tokens[1]
                move_motors(a, five)

            elif cmd == "edit" and len(tokens) >= 3:
                edit_pose(a, tokens[1:])

            elif cmd == "free":
                a.release()
                print("  arm torque OFF (gripper still on). Move the arm by hand, then 'save NAME' or 'hold'.")

            elif cmd == "hold":
                a.hold()
                show(a)

            elif cmd == "freegrip":
                a.release_gripper()
                print("  GRIPPER torque OFF. Move the claw by hand, then 'savegrip NAME' or 'holdgrip'.")
                if a.holding:
                    print(f"  (the {a.holding} may drop)")

            elif cmd == "holdgrip":
                a.hold_gripper()
                show(a)

            elif cmd == "save" and len(tokens) == 2:
                name = tokens[1].lower()
                if name in arm_lib.grip_names():
                    print(f"  {name} is a gripper name. Did you mean 'savegrip {name}'?")
                    continue
                if name not in arm_lib.valid_pose_names():
                    print(f"  '{name}' isn't a valid pose name. Nothing saved.")
                    print("  " + arm_lib.POSE_NAME_HELP)
                    continue
                a.refresh()
                old = a.poses["arm"].get(name)
                a.poses["arm"][name] = list(a.current)
                arm_lib.save_poses(a.poses)
                print(f"  saved {name} = {a.current}" + (f"   (was {old})" if old else ""))
                warn_limits(dict(zip(hw.ARM_JOINTS, a.current)))

            elif cmd == "savegrip" and len(tokens) == 2:
                name = tokens[1].lower()
                if name not in arm_lib.grip_names():
                    print("  grip names: o1 o2 o3 (open), c1 c2 c3 (closed); 1 trophy, 2 cube, 3 cylinder")
                    continue
                a.refresh()   # if the gripper was moved by hand, save where it really is
                old = a.poses["grip"].get(name)
                a.poses["grip"][name] = a.grip_pos
                arm_lib.save_poses(a.poses)
                print(f"  saved grip {name} = {a.grip_pos}" + (f"   (was {old})" if old is not None else ""))
                warn_limits({hw.GRIPPER: a.grip_pos})

            elif cmd == "go" and len(tokens) == 2:
                a.go(tokens[1])
                show(a)

            elif cmd == "grip" and len(tokens) == 2:
                a.grip(tokens[1].lower())

            elif cmd == "pick" and len(tokens) == 2:
                if a.holding:
                    print(f"  already holding a {a.holding}; 'place C' or open the gripper first")
                    continue
                a.pick(int(tokens[1]))

            elif cmd == "place" and len(tokens) == 2:
                a.place(int(tokens[1]))

            elif cmd == "run" and len(tokens) == 3:
                a.pick_and_place(int(tokens[1]), int(tokens[2]))

            elif cmd == "attack" and len(tokens) == 2:
                a.attack(int(tokens[1]))

            elif cmd == "watch":
                watch(a)

            elif cmd == "speed" and len(tokens) == 2:
                a.set_speed(tokens[1].lower())
                sp = arm_lib.SPEEDS[a.speed_name]
                print(f"  {a.speed_name} speed: travel {sp['travel']}, fine {sp['fine']}, "
                      f"accel {sp['accel']}, grip wait {sp['grip_wait']}s")

            elif cmd == "show":
                show(a)

            elif cmd == "limits":
                for i, name in enumerate(hw.ALL_JOINTS, start=1):
                    lo, hi = hw.JOINT_LIMITS[name]
                    print(f"  motor {i} {name:9s} {lo:4d} - {hi:4d}")
                print("  (edit JOINT_LIMITS in hardware.py, then restart teach mode)")

            elif cmd == "list":
                valid = set(arm_lib.valid_pose_names())
                for name, v in sorted(a.poses["arm"].items()):
                    note = "" if name in valid else "   (old name, never used)"
                    print(f"  {name:20s} {' '.join(str(x) for x in v)}{note}")
                for name, v in sorted(a.poses["grip"].items()):
                    print(f"  grip {name:15s} {v}")

            elif cmd == "missing":
                need = [n for n in arm_lib.required_poses() if n not in a.poses["arm"]]
                need += [f"grip:{g}" for g in arm_lib.required_grips() if g not in a.poses["grip"]]
                print("  " + (", ".join(need) if need else "nothing, all recorded"))

            elif cmd == "points" and len(tokens) == 2:
                for i, p in enumerate(load_sequence_points(tokens[1]), start=1):
                    values = " ".join(str(p[n]) for n in hw.ALL_JOINTS)
                    print(f"  {i:2d}: {values}")

            elif cmd == "take" and len(tokens) == 4:
                points = load_sequence_points(tokens[1])
                n = int(tokens[2])
                if not 1 <= n <= len(points):
                    print(f"  that file has points 1-{len(points)}")
                    continue
                p = points[n - 1]
                name = tokens[3].lower()
                if name not in arm_lib.valid_pose_names():
                    print(f"  '{name}' isn't a valid pose name. Nothing saved.")
                    print("  " + arm_lib.POSE_NAME_HELP)
                    continue
                a.poses["arm"][name] = [p[j] for j in hw.ARM_JOINTS]
                arm_lib.save_poses(a.poses)
                print(f"  saved {name} = {a.poses['arm'][name]}   (gripper in that point was {p[hw.GRIPPER]})")
                warn_limits({j: p[j] for j in hw.ARM_JOINTS})

            elif cmd == "help":
                print(HELP)

            elif cmd in ("quit", "exit", "q"):
                break

            else:
                print("  didn't understand that, type 'help'")

        except hw.StopPressed:
            print("\n  Stopped. Arm is holding still.")
        except KeyError as e:
            print(f"  {e.args[0]}")
        except (ValueError, RuntimeError) as e:
            print(f"  {e}")
        except ImportError:
            print("  angle files need the course code (arm_helpers.py) next to the game folder")

    if not hw.DRY_RUN:
        print("Leaving teach mode. Torque stays " + ("OFF." if a.free else "ON (arm holding)."))
    a.close()


if __name__ == "__main__":
    main()
