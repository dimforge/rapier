"""Command-line scenario picker for the Rapier Python testbed.

The interactive picker prints the list of bundled examples, prompts on
stdin for an index, and loads/unloads testbeds inside a single Panda3D
``ShowBase`` event loop. Key design notes:

* **Single ``ShowBase`` for the lifetime of the process.** Panda3D on
  macOS can't host more than one ``ShowBase``; reusing one is also
  faster.
* **The Panda3D task loop runs forever on the main thread.** This is the
  only thread that may touch the scene graph and the only thread Cocoa
  allows to pump UI events. Without it the window goes unresponsive.
* **stdin reading runs on a background daemon thread.** It queues
  commands (``launch``, ``quit``) to the main thread.
* **Esc unloads the testbed but doesn't stop the loop.** The window
  stays open for the next pick. The user types ``q`` (or hits Ctrl-D,
  or closes the window) to exit the program.

Headless mode (``PANDA_NO_WINDOW=1``) skips everything and runs the
first registered example for a fixed number of steps — what CI uses.
"""

from __future__ import annotations

import os
import queue
import shutil
import sys
import threading
from typing import Iterable, List, Optional, Tuple

from ._registry import EXAMPLES, Example, by_label
from ._testbed import Testbed, _is_headless


def _group(examples: Iterable[Example]) -> List[Tuple[str, List[Example]]]:
    """Group examples by category (preserving first-seen order)."""
    out: List[Tuple[str, List[Example]]] = []
    by_cat: "dict[str, List[Example]]" = {}
    for ex in examples:
        by_cat.setdefault(ex.category, []).append(ex)
        if all(cat != ex.category for cat, _ in out):
            out.append((ex.category, by_cat[ex.category]))
    return out


def _menu_order(examples: Iterable[Example]) -> List[Example]:
    """Flatten the grouped menu into the order indices refer to.

    The menu prints examples grouped by category. When the user types
    ``N``, that ``N`` indexes into THIS list — not into the raw
    registration order, which interleaves categories.
    """
    return [ex for _cat, grp in _group(examples) for ex in grp]


def run(initial: Optional[str] = None) -> None:
    """Entry point used by ``python -m rapier_testbed[.<name>]``."""
    from . import _autoload_bundled_examples

    _autoload_bundled_examples()

    if not EXAMPLES:
        print("No examples registered yet.", file=sys.stderr)
        return

    if initial is not None:
        ex = by_label(initial)
        if ex is None:
            print(f"Unknown example: {initial!r}", file=sys.stderr)
            sys.exit(2)
        _launch_single(ex)
        return

    if _is_headless():
        menu = _menu_order(EXAMPLES)
        _print_menu(menu)
        _launch_single(menu[0])
        return

    _cli_loop()


# ---------------------------------------------------------------------------
# Direct (single-example) launch — used by ``python -m rapier_testbed.<name>``
# and the headless smoke tests. Calls ``Testbed.run()`` and exits when the
# user presses Esc or closes the window.
# ---------------------------------------------------------------------------


def _launch_single(example: Example) -> None:
    if _is_headless():
        tb = Testbed(headless=True)
        tb.load_example(example.init_fn, f"{example.category} / {example.name}")
        tb.run()
        return

    from ._testbed import _ensure_panda_config
    from direct.showbase.ShowBase import ShowBase

    _ensure_panda_config()
    base = ShowBase()
    base.userExit = lambda: base.taskMgr.stop()  # type: ignore[union-attr]
    base.finalizeExit = lambda: None  # type: ignore[union-attr]
    tb = Testbed(base=base)
    tb.load_example(example.init_fn, f"{example.category} / {example.name}")
    tb.run()


# ---------------------------------------------------------------------------
# Interactive CLI loop
# ---------------------------------------------------------------------------


def _cli_loop() -> None:
    """Run the picker until the user types ``q`` or closes the window.

    Architecture:
        Main thread     – ``ShowBase`` task loop runs forever (the only
                          thread Cocoa allows to pump UI events). A poll
                          task drains a command queue each frame: on
                          ``launch`` it opens the window the first time
                          and constructs a fresh Testbed; on ``quit`` it
                          stops the main loop.
        CLI thread      – Daemon. Blocks on ``input()``, queues commands.

    User-facing behavior:
        * The window opens lazily on the first pick — nothing appears at
          startup.
        * Subsequent picks reuse the same window (loading new scene
          content into it). The window never closes mid-run; doing so
          on macOS Cocoa is unreliable.
        * **Esc** unloads the current sim and keeps the CLI alive (the
          window stays open, showing the cream background).
        * **The window's X button** quits the program (equivalent to
          typing ``q`` in the terminal).
        * **`q` / Ctrl-D** quits.
    """
    from ._testbed import _ensure_panda_config
    from direct.showbase.ShowBase import ShowBase
    from direct.task import Task

    _ensure_panda_config()
    # No initial main window — the user will trigger one with their
    # first pick. Avoids a flash of grey/empty window at startup.
    base = ShowBase(windowType="none")
    # Panda3D's default ``userExit`` calls ``sys.exit()``. Replace with
    # a soft no-op so neither a stray internal call nor (with default
    # window-event wiring) a close-button click can take down the CLI.
    base.userExit = lambda: None  # type: ignore[union-attr]
    base.finalizeExit = lambda: None  # type: ignore[union-attr]

    cmd_queue: "queue.Queue[Tuple[str, Optional[Example]]]" = queue.Queue()
    unload_request = threading.Event()
    quit_event = threading.Event()

    # Mutable holder so the closures below can rebind the active testbed.
    state: "dict[str, Optional[Testbed]]" = {"testbed": None}

    def _on_testbed_escape() -> None:
        # Runs on the main thread (Panda3D event callback). Just toggle
        # the flag; the polling task does the actual unload next frame
        # so we don't tear down state from inside a callback.
        unload_request.set()

    def _unload_current() -> None:
        tb = state["testbed"]
        if tb is None:
            return
        try:
            tb._cleanup_scene()
        except Exception:
            pass
        state["testbed"] = None

    def _bind_close_button() -> None:
        """Wire the window's X button to enqueue a ``quit`` command.

        Calling ``closeWindow`` mid-run on macOS Cocoa is unreliable
        (the destroy is queued through the graphics engine but Cocoa's
        event pump needs the main loop to flush it, and trying to do so
        synchronously from inside a task callback deadlocks). Instead,
        clicking the X just behaves like typing ``q`` in the terminal:
        the main loop processes the quit command on its next tick,
        ``base.run()`` returns cleanly, and Python's normal teardown
        closes the window from outside the loop.

        Must be re-called after ``Testbed.__init__`` since the testbed
        clears ``base.ignoreAll()`` first.
        """
        if base.win is None:
            return

        def _on_x() -> None:
            cmd_queue.put(("quit", None))

        try:
            base.win.setCloseRequestEvent("rapier-cli-window-close")
            base.accept("rapier-cli-window-close", _on_x)
        except Exception:
            pass

    def _load(example: Example) -> None:
        _unload_current()
        # Open the window on demand the first time. Subsequent picks
        # reuse the same window — we never close it mid-run.
        if base.win is None:
            try:
                base.openMainWindow()
            except Exception as e:
                print(f"  (couldn't open window: {e!r})", file=sys.stderr)
                return
        tb = Testbed(base=base, on_escape=_on_testbed_escape)
        tb.load_example(example.init_fn, f"{example.category} / {example.name}")
        tb.install_step_task()
        # Testbed.__init__ calls ``base.ignoreAll()`` so we re-wire the X
        # button AFTER constructing it.
        _bind_close_button()
        state["testbed"] = tb

    def _poll(task: object) -> object:
        # Drain pending CLI commands.
        while True:
            try:
                cmd, payload = cmd_queue.get_nowait()
            except queue.Empty:
                break
            if cmd == "quit":
                quit_event.set()
                _unload_current()
                base.taskMgr.stop()
                return Task.done
            if cmd == "launch" and payload is not None:
                _load(payload)
        # Esc inside the window unloads the current testbed and leaves
        # the window open (cream background) so the user can pick
        # another scenario without re-opening anything.
        if unload_request.is_set():
            unload_request.clear()
            _unload_current()
        return Task.cont

    base.taskMgr.add(_poll, "rapier-cli-poll")

    menu = _menu_order(EXAMPLES)
    cli_thread = threading.Thread(
        target=_cli_input_loop,
        args=(cmd_queue, quit_event, menu),
        name="rapier-cli",
        daemon=True,
    )
    cli_thread.start()

    try:
        base.run()
    finally:
        quit_event.set()
        _unload_current()


def _cli_input_loop(
    cmd_queue: "queue.Queue[Tuple[str, Optional[Example]]]",
    quit_event: threading.Event,
    examples: List[Example],
) -> None:
    """Background thread: read stdin, push commands.

    ``examples`` is the flat menu-ordered list — i.e. ``examples[idx-1]``
    is what the user means when they type ``idx``.
    """
    _print_menu(examples)
    last: Optional[int] = None
    n = len(examples)
    while not quit_event.is_set():
        prompt = "\nExample # (Enter=repeat, m=menu, q=quit): "
        if last is not None:
            prompt = f"\nExample # [{last}] (Enter=repeat, m=menu, q=quit): "
        try:
            raw = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            cmd_queue.put(("quit", None))
            return

        if quit_event.is_set():
            return

        cmd = raw.lower()
        if cmd in ("q", "quit", "exit"):
            cmd_queue.put(("quit", None))
            return
        if cmd in ("m", "menu", "?", "h", "help"):
            _print_menu(examples)
            continue
        if not raw:
            if last is None:
                continue
            idx = last
        else:
            try:
                idx = int(raw)
            except ValueError:
                print(f"  (not an integer: {raw!r}; type 'm' for the menu)")
                continue

        if not (1 <= idx <= n):
            print(f"  (out of range: 1..{n})")
            continue

        ex = examples[idx - 1]
        last = idx
        print(f"  → {idx:>3}.  {ex.category} / {ex.name}")
        print("     (Esc in the window = back to CLI; close window = quit)")
        cmd_queue.put(("launch", ex))


# ---------------------------------------------------------------------------
# Menu formatting
# ---------------------------------------------------------------------------


def _print_menu(examples: List[Example]) -> None:
    """Print the example list in multi-column form, grouped by category."""
    n_total = len(examples)
    print()
    print(f"Rapier Python Testbed — {n_total} examples")
    print("=" * 60)

    idx_width = max(2, len(str(n_total)))
    name_width = max(len(ex.name) for ex in examples)
    # Cell layout: "  NN.  <name padded to name_width>" + trailing pad.
    cell_width = idx_width + 2 + name_width + 4

    try:
        term_w = shutil.get_terminal_size((100, 24)).columns
    except Exception:
        term_w = 100
    cols = max(1, min(4, term_w // max(cell_width, 1)))

    cur_idx = 1
    for cat, group in _group(examples):
        print()
        print(f"  [ {cat} ]")
        row: List[str] = []
        for ex in group:
            cell = f"  {cur_idx:>{idx_width}}.  {ex.name:<{name_width}}"
            row.append(cell)
            cur_idx += 1
            if len(row) == cols:
                print("    " + "".join(c.ljust(cell_width) for c in row))
                row = []
        if row:
            print("    " + "".join(c.ljust(cell_width) for c in row))

    print()
    print(f"Total: {n_total} scenarios.")


__all__ = ["run"]
