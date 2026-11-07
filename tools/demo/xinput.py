"""Minimal xdotool stand-in for record_demo.sh (python-xlib + XTest, no window manager needed).

xinput.py place NAME X Y W H    move/resize the first mapped top-level window whose title has NAME
xinput.py click X Y
xinput.py drag X0 Y0 X1 Y1      left-button drag in small steps
xinput.py type TEXT
xinput.py key KEY [KEY...]      X keysym names, modifiers joined with "+": Return, ctrl+a
"""

import sys
import time
from collections.abc import Iterator

from Xlib import XK, X, display
from Xlib.ext import xtest

d = display.Display()
MODS = {"ctrl": "Control_L", "shift": "Shift_L", "alt": "Alt_L"}
NAMES = {" ": "space", ".": "period", "-": "minus", "/": "slash", "_": "underscore", ":": "colon"}


def windows(w: object, depth: int = 0) -> Iterator[tuple[object, str]]:
    """(window, title) of the mapped windows down to the grandchildren of w."""
    for c in w.query_tree().children:
        try:
            name = c.get_wm_name()
            mapped = c.get_attributes().map_state == X.IsViewable
        except Exception:  # noqa: BLE001 (a window can vanish while walking the tree)
            continue
        if name and mapped:
            yield c, str(name)
        if depth < 2:
            yield from windows(c, depth + 1)


def pointer(x: float | str, y: float | str) -> None:
    xtest.fake_input(d, X.MotionNotify, x=int(x), y=int(y))
    d.sync()


def button(down: bool) -> None:
    xtest.fake_input(d, X.ButtonPress if down else X.ButtonRelease, 1)
    d.sync()


def keycode(name: str) -> int:
    sym = XK.string_to_keysym(name)
    return d.keysym_to_keycode(sym if sym else ord(name))


def stroke(names: list[str]) -> None:
    """Press names in order, release in reverse."""
    codes = [keycode(n) for n in names]
    for c in codes:
        xtest.fake_input(d, X.KeyPress, c)
    for c in reversed(codes):
        xtest.fake_input(d, X.KeyRelease, c)
    d.sync()
    time.sleep(0.04)


def main(cmd: str, *args: str) -> int:
    if cmd == "place":
        x, y, w, h = map(int, args[1:5])
        for win, name in windows(d.screen().root):
            if args[0] in name:
                win.configure(x=x, y=y, width=w, height=h)
                d.sync()
                return 0
        return 1
    if cmd == "click":
        pointer(args[0], args[1])
        time.sleep(0.1)
        button(True)
        time.sleep(0.06)
        button(False)
    elif cmd == "drag":
        x0, y0, x1, y1 = map(int, args)
        pointer(x0, y0)
        time.sleep(0.2)
        button(True)
        for i in range(1, 41):
            pointer(x0 + (x1 - x0) * i / 40, y0 + (y1 - y0) * i / 40)
            time.sleep(0.025)
        time.sleep(0.1)
        button(False)
    elif cmd == "type":
        for ch in args[0]:
            name = NAMES.get(ch, ch)
            stroke(["Shift_L", name] if ch.isupper() or ch in "_:" else [name])
    elif cmd == "key":
        for combo in args:
            *mods, key = combo.split("+")
            stroke([MODS[m] for m in mods] + [key])
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
