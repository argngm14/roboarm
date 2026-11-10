"""
game.py - the console you run during a match. Game speed, and it tracks
which blocks have been used so the same block can't be picked twice.

On the Pi, from the STS3215-5DOF-Robot-Arm folder:
    source .venv/bin/activate
    python game/game.py

Cells (from your side):   1 2 3
                          4 5 6
                          7 8 9
"""

import re

try:
    import readline  # noqa: F401  (up-arrow command history)
except ImportError:
    pass

import arm as arm_lib
import hardware as hw

# Pattern commands: which cells each one fills.
PATTERNS = {
    "/":  [7, 5, 3],              # diagonal, bottom-left to top-right
    "\\": [1, 5, 9],              # diagonal, top-left to bottom-right
    "_1": [1, 2, 3],              # top row
    "_2": [4, 5, 6],              # middle row
    "_3": [7, 8, 9],              # bottom row
    "|1": [1, 4, 7],              # left column
    "|2": [2, 5, 8],              # middle column
    "|3": [3, 6, 9],              # right column
    "=":  [1, 2, 3, 4, 5, 6],     # top two rows
    "||": [1, 4, 7, 3, 6, 9],     # both outside columns
}

HELP = """
  1 5          pick block 1 and place it in cell 5
  k8           attack cell 8 (asks for the referee's OK first)
  PATTERN      for fun, on an EMPTY rack: fill a pattern, highest-point blocks first
                 /  \\          diagonals
                 _1 _2 _3      rows (top, middle, bottom)
                 |1 |2 |3      columns (left, middle, right)
                 =             top two rows
                 ||            both outside columns
               Cells are filled top to bottom, left to right. If there aren't
               enough blocks left, it fills what it can and stops.
  used         which blocks are used / still available
  unuse B      block B is back on its plate (e.g. after a retry), so it can be picked again
  reset        new game: every block available again
  safe         move to the safe pose
  speed test|game
  help / quit                (Space during any move = stop and hold)

  Cells
    1 2 3
    4 5 6
    7 8 9
"""

ATTACK = re.compile(r"^k([1-9])$")


def show_blocks(a):
    used = sorted(a.tracker.used)
    avail = a.tracker.available()
    print(f"  used: {used if used else 'none'}   available: {avail if avail else 'none'}")


def blocks_high_to_low(a):
    """Available blocks, most points first; same type in block-number order."""
    return sorted(
        a.tracker.available(),
        key=lambda b: (-arm_lib.TYPE_POINTS[arm_lib.BLOCK_TYPES[b]], b),
    )


def plan_pattern(a, symbol):
    cells = sorted(PATTERNS[symbol])  # top to bottom, left to right
    blocks = blocks_high_to_low(a)
    return list(zip(blocks, cells)), cells


def run_pattern(a, symbol):
    """For fun, on an empty rack. Fills what it can with the blocks left, then stops."""
    plan, cells = plan_pattern(a, symbol)
    if not plan:
        print("  no blocks left. Nothing moved.")
        return
    for block, cell in plan:          # check every route before the first move
        a.check_route(block, cell)

    steps = ", ".join(f"{b} ({arm_lib.BLOCK_TYPES[b]}) -> c{c}" for b, c in plan)
    print(f"  plan: {steps}")
    if len(plan) < len(cells):
        empty = ", ".join(f"c{c}" for c in cells[len(plan):])
        print(f"  only {len(plan)} block(s) left, so {empty} stays empty")

    for i, (block, cell) in enumerate(plan, start=1):
        print(f"  [{i}/{len(plan)}]")
        a.pick_and_place(block, cell)
    print(f"  done: filled {len(plan)} of {len(cells)} cells")


def main():
    a = arm_lib.Arm(speed="game", track=True)

    if hw.DRY_RUN:
        print("=== PRACTICE MODE: motors are NOT moving ===")
    else:
        print("Connected. The arm is holding where it is; nothing has moved.")
    print("Game speed. Space during a move = stop and hold.")
    show_blocks(a)
    if a.tracker.used:
        print("  Starting a new game? Type 'reset' first.")
    print("Type 'help' for commands.")

    while True:
        try:
            line = input("game> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        tokens = line.split()
        cmd = tokens[0]

        try:
            if len(tokens) == 2 and tokens[0].isdigit() and tokens[1].isdigit():
                a.pick_and_place(int(tokens[0]), int(tokens[1]))
                show_blocks(a)

            elif ATTACK.match(cmd) and len(tokens) == 1:
                cell = int(ATTACK.match(cmd).group(1))
                answer = input(f"  Attack c{cell}: call 'Attack', press Enter when the referee "
                               f"authorizes (n to cancel): ").strip().lower()
                if answer == "n":
                    print("  cancelled")
                    continue
                a.attack(cell)

            elif cmd in PATTERNS and len(tokens) == 1:
                run_pattern(a, cmd)
                show_blocks(a)

            elif cmd == "used":
                show_blocks(a)

            elif cmd == "unuse" and len(tokens) == 2:
                a.tracker.unmark(int(tokens[1]))
                show_blocks(a)

            elif cmd == "reset":
                a.tracker.reset()
                print("  new game: all blocks available")

            elif cmd == "safe":
                a.go("safe")

            elif cmd == "speed" and len(tokens) == 2:
                a.set_speed(tokens[1])
                print(f"  {a.speed_name} speed")

            elif cmd == "help":
                print(HELP)

            elif cmd in ("quit", "exit", "q"):
                break

            else:
                print("  didn't understand that, type 'help'")

        except hw.StopPressed:
            print("\n  Stopped. Arm is holding still.")
            show_blocks(a)
        except KeyError as e:
            print(f"  {e.args[0]}")
        except (ValueError, RuntimeError) as e:
            print(f"  {e}")

    a.close()


if __name__ == "__main__":
    main()
