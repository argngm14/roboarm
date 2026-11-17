"""
console.py - teach mode and game mode in one console, so you can record a pose,
test it at game speed and adjust it again without quitting.

On the Pi, from the STS3215-5DOF-Robot-Arm folder:
    source .venv/bin/activate
    python game/console.py          (starts in teach mode)
    python game/console.py game     (starts in game mode)

Switch any time with:
    mode teach    record and test poses: test speed, every teach command
    mode game     play: game speed, block tracking, the command queue

The arm stays connected and holding when you switch. Each mode keeps its own
commands (in teach mode "1 +10" nudges motor 1; in game mode "1 5" picks block 1
into cell 5), so the prompt always shows which mode you are in.
teach.py and game.py still work on their own, as before.
"""

import sys

import arm as arm_lib
import game
import hardware as hw
import teach

MODES = {"teach": teach, "game": game}

MODE_HELP = """
  mode teach       switch to teach mode (test speed, record and adjust poses)
  mode game        switch to game mode (game speed, block tracking, queue)
  mode             which mode you are in
"""


def switch(a, name):
    MODES[name].enter(a)
    if name == "game":
        print("  GAME mode: game speed, blocks are tracked, you can queue the next command during a move")
        game.show_blocks(a)
    else:
        print("  TEACH mode: test speed, all teach commands")
    if a.free:
        print("  (arm torque is OFF from 'free'; the next move turns it back on)")
    return name


def main():
    start = sys.argv[1].lower() if len(sys.argv) > 1 else "teach"
    if start not in MODES:
        print(f"usage: python game/console.py [{'|'.join(MODES)}]")
        sys.exit(1)

    a = arm_lib.Arm(speed="test")
    teach.banner(a)
    mode = switch(a, start)
    print("Type 'help' for commands, 'mode game' / 'mode teach' to switch.")

    while True:
        prompt = f"{mode}> "
        try:
            line = game.next_line(prompt) if mode == "game" else input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue

        tokens = line.lower().split()
        if tokens[0] == "mode":
            if len(tokens) == 1:
                print(f"  {mode} mode")
            elif tokens[1] in MODES:
                if tokens[1] == mode:
                    print(f"  already in {mode} mode")
                else:
                    mode = switch(a, tokens[1])
            else:
                print("  use 'mode teach' or 'mode game'")
            continue

        if not MODES[mode].handle(a, line):
            break
        if tokens[0] == "help":
            print(MODE_HELP)

    teach.goodbye(a)
    a.close()


if __name__ == "__main__":
    main()
