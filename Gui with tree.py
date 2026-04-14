"""
Elevator Optimization System — Single Elevator + Search Tree Viewer
====================================================================
Run this file in PyCharm.  No pip installs needed.

Main window  : building animation, algorithm selection, results panel
Tree Viewer  : opens as a second window via the "View Search Tree" button.
               Shows exactly how the chosen algorithm traverses the state
               space for the current start + request configuration.
               Step through manually or press Play.
"""

import tkinter as tk
from tkinter import messagebox
import time
import threading
import heapq
from collections import deque
from elevator import bfs, dfs, ucs, astar, path_total_cost


# ═══════════════════════════════════════════════════════════════════════════════
#  Theme
# ═══════════════════════════════════════════════════════════════════════════════

BG       = "#0f1117"
PANEL    = "#1a1d27"
CARD     = "#22263a"
ACCENT   = "#4f8ef7"
ACCENT2  = "#38d9a9"
WARN     = "#f7a84f"
DANGER   = "#f75f5f"
TEXT     = "#e8eaf0"
DIM      = "#7c82a0"
SHAFT    = "#2a2f45"
FLOOR_H  = 42

ALGORITHMS = {
    "BFS  (Breadth-First)": bfs,
    "DFS  (Depth-First)":   dfs,
    "UCS  (Uniform Cost)":  ucs,
    "A*   (A-Star)":        astar,
}
ALGO_COLORS = {
    "BFS  (Breadth-First)": WARN,
    "DFS  (Depth-First)":   DANGER,
    "UCS  (Uniform Cost)":  ACCENT,
    "A*   (A-Star)":        ACCENT2,
}
ALGO_INFO = {
    "BFS  (Breadth-First)":  "Explores by fewest stops. Not cost-optimal.",
    "DFS  (Depth-First)":    "Dives deep first. Fast but often suboptimal.",
    "UCS  (Uniform Cost)":   "Expands cheapest path. Guarantees optimal cost.",
    "A*   (A-Star)":         "Cost + heuristic. Optimal and most efficient.",
}

# Node state colors in the tree
NODE_UNVISITED = "#444455"
NODE_FRONTIER  = "#1a4a7a"
NODE_EXPLORED  = "#0f4a35"
NODE_CURRENT   = ""          # filled per-algo
NODE_GOAL      = "#7a1a1a"

NODE_UNVISITED_BORDER = "#888899"
NODE_FRONTIER_BORDER  = "#378ADD"
NODE_EXPLORED_BORDER  = "#1D9E75"
NODE_GOAL_BORDER      = "#E24B4A"

TEXT_UNVISITED = "#aaaacc"
TEXT_FRONTIER  = "#85B7EB"
TEXT_EXPLORED  = "#5DCAA5"
TEXT_GOAL      = "#F09595"
TEXT_CURRENT   = "#ffffff"

EDGE_DEFAULT  = "#3a3a55"
EDGE_PATH     = "#EF9F27"
EDGE_PATH_W   = 2.5


# ═══════════════════════════════════════════════════════════════════════════════
#  Tree step generator — produces a list of snapshots for each algorithm
# ═══════════════════════════════════════════════════════════════════════════════

def _state_key(floor, remaining):
    return f"{floor}|{','.join(str(f) for f in sorted(remaining))}"


def _path_cost(path):
    return sum(abs(path[i] - path[i - 1]) for i in range(1, len(path)))


def generate_steps(algo: str, start: int, requests: list):
    """
    Returns (tree_nodes, tree_edges, steps).

    tree_nodes : dict  key -> {floor, remaining, depth}
    tree_edges : list  of {from_key, to_key, cost}
    steps      : list  of {current_key, frontier_keys, explored_keys,
                            goal_key, solution_path, message}

    All possible states are pre-built as a tree (depth-first expansion),
    then we replay the algorithm traversal as steps over that tree.
    The tree is capped at 4 requests to keep layout manageable.
    """

    reqs = requests[:4]   # safety cap for display

    # ── 1. Build full tree of all states ──────────────────────────────────────
    nodes = {}
    edges = []

    def expand(floor, rem_tuple, depth, parent_key):
        key = _state_key(floor, rem_tuple)
        if key in nodes:
            return
        nodes[key] = {"floor": floor, "remaining": list(rem_tuple), "depth": depth, "key": key}
        if parent_key:
            c = abs(floor - int(parent_key.split("|")[0]))
            edges.append({"from": parent_key, "to": key, "cost": c})
        for f in rem_tuple:
            new_rem = tuple(x for x in rem_tuple if x != f)
            expand(f, new_rem, depth + 1, key)

    expand(start, tuple(sorted(reqs)), 0, None)

    # ── 2. Replay algorithm, generating snapshots ─────────────────────────────
    steps = []

    def snap(current, frontier, explored, goal_key, sol_path, msg):
        steps.append({
            "current_key":   current,
            "frontier_keys": set(frontier),
            "explored_keys": set(explored),
            "goal_key":      goal_key,
            "solution_path": list(sol_path),
            "message":       msg,
        })

    initial_key = _state_key(start, sorted(reqs))
    snap(None, {initial_key}, set(), None, [],
         f"Initial state: elevator at floor {start}, requests {sorted(reqs)}.")

    if algo == "BFS":
        queue   = deque([{"floor": start, "rem": frozenset(reqs), "path": [start]}])
        visited = set()
        frontier_keys = {initial_key}
        explored_keys = set()

        while queue:
            node = queue.popleft()
            key  = _state_key(node["floor"], sorted(node["rem"]))
            frontier_keys.discard(key)

            if key in visited:
                continue
            visited.add(key)
            explored_keys.add(key)

            snap(key, frontier_keys, explored_keys, None, node["path"],
                 f"BFS pops: floor={node['floor']}, remaining={sorted(node['rem'])}. "
                 f"Nodes explored so far: {len(explored_keys)}.")

            if not node["rem"]:
                snap(key, frontier_keys, explored_keys, key, node["path"],
                     f"Goal reached! Path: {node['path']}. "
                     f"Cost: {_path_cost(node['path'])}. Nodes explored: {len(explored_keys)}.")
                return nodes, edges, steps

            for f in sorted(node["rem"]):
                child = {"floor": f,
                         "rem":   node["rem"] - {f},
                         "path":  node["path"] + [f]}
                ck = _state_key(f, sorted(child["rem"]))
                if ck not in visited and ck not in frontier_keys:
                    frontier_keys.add(ck)
                    queue.append(child)
                    snap(key, frontier_keys, explored_keys, None, node["path"],
                         f"BFS enqueues floor={f} "
                         f"(step cost={abs(node['floor']-f)}). "
                         f"Queue length: {len(queue)}.")

    elif algo == "DFS":
        stack   = [{"floor": start, "rem": frozenset(reqs), "path": [start]}]
        visited = set()
        frontier_keys = {initial_key}
        explored_keys = set()

        while stack:
            node = stack.pop()
            key  = _state_key(node["floor"], sorted(node["rem"]))
            frontier_keys.discard(key)

            if key in visited:
                continue
            visited.add(key)
            explored_keys.add(key)

            snap(key, frontier_keys, explored_keys, None, node["path"],
                 f"DFS pops (stack top): floor={node['floor']}, "
                 f"remaining={sorted(node['rem'])}. "
                 f"Nodes explored: {len(explored_keys)}.")

            if not node["rem"]:
                snap(key, frontier_keys, explored_keys, key, node["path"],
                     f"Goal reached! Path: {node['path']}. "
                     f"Cost: {_path_cost(node['path'])}. Nodes explored: {len(explored_keys)}.")
                return nodes, edges, steps

            for f in sorted(node["rem"], reverse=True):
                child = {"floor": f,
                         "rem":   node["rem"] - {f},
                         "path":  node["path"] + [f]}
                ck = _state_key(f, sorted(child["rem"]))
                if ck not in visited:
                    frontier_keys.add(ck)
                    stack.append(child)
                    snap(key, frontier_keys, explored_keys, None, node["path"],
                         f"DFS pushes floor={f} onto stack "
                         f"(step cost={abs(node['floor']-f)}).")

    elif algo in ("UCS", "A*"):
        def h(floor, rem):
            if not rem:
                return 0
            return min(abs(floor - f) for f in rem)

        counter  = [0]
        heap     = []
        use_h    = (algo == "A*")
        g0       = 0
        node0    = {"floor": start, "rem": frozenset(reqs), "path": [start], "g": 0}
        f0       = g0 + (h(start, frozenset(reqs)) if use_h else 0)
        heapq.heappush(heap, (f0, counter[0], node0))
        visited       = {}
        frontier_keys = {initial_key}
        explored_keys = set()

        while heap:
            f_val, _, node = heapq.heappop(heap)
            key = _state_key(node["floor"], sorted(node["rem"]))
            frontier_keys.discard(key)

            if key in visited and visited[key] <= node["g"]:
                continue
            visited[key] = node["g"]
            explored_keys.add(key)

            hval = h(node["floor"], node["rem"])
            if use_h:
                msg = (f"A* pops: floor={node['floor']}, "
                       f"remaining={sorted(node['rem'])}, "
                       f"g={node['g']}, h={hval}, f={node['g']+hval}. "
                       f"Nodes: {len(explored_keys)}.")
            else:
                msg = (f"UCS pops cheapest: floor={node['floor']}, "
                       f"remaining={sorted(node['rem'])}, "
                       f"cost={node['g']}. Nodes: {len(explored_keys)}.")
            snap(key, frontier_keys, explored_keys, None, node["path"], msg)

            if not node["rem"]:
                snap(key, frontier_keys, explored_keys, key, node["path"],
                     f"Goal! Path: {node['path']}. "
                     f"Cost: {node['g']}. Nodes: {len(explored_keys)}.")
                return nodes, edges, steps

            for f in node["rem"]:
                child_g   = node["g"] + abs(node["floor"] - f)
                child_rem = node["rem"] - {f}
                child     = {"floor": f, "rem": child_rem,
                             "path":  node["path"] + [f], "g": child_g}
                ck = _state_key(f, sorted(child_rem))
                if ck not in visited or visited[ck] > child_g:
                    ch_h = h(f, child_rem) if use_h else 0
                    counter[0] += 1
                    heapq.heappush(heap, (child_g + ch_h, counter[0], child))
                    frontier_keys.add(ck)
                    if use_h:
                        add_msg = (f"A* adds floor={f}: "
                                   f"g={child_g}, h={ch_h}, f={child_g+ch_h}.")
                    else:
                        add_msg = f"UCS adds floor={f}: cost={child_g}."
                    snap(key, frontier_keys, explored_keys, None, node["path"], add_msg)

    return nodes, edges, steps


# ═══════════════════════════════════════════════════════════════════════════════
#  Tree layout  — places nodes on a 2-D canvas
# ═══════════════════════════════════════════════════════════════════════════════

NODE_R   = 24     # circle radius
LEVEL_H  = 90     # vertical distance between depth levels
CANVAS_W = 900    # logical SVG/canvas width


def compute_layout(nodes: dict, edges: list):
    """
    Returns positions dict: key -> (x, y)
    Nodes are grouped by depth level and spread evenly across CANVAS_W.
    """
    by_depth = {}
    for key, nd in nodes.items():
        d = nd["depth"]
        by_depth.setdefault(d, []).append(key)

    positions = {}
    max_depth  = max(by_depth) if by_depth else 0
    canvas_h   = (max_depth + 1) * LEVEL_H + 60

    for depth, keys in by_depth.items():
        n     = len(keys)
        y     = 50 + depth * LEVEL_H
        for i, key in enumerate(keys):
            x = CANVAS_W * (i + 1) / (n + 1)
            positions[key] = (x, y)

    return positions, canvas_h


# ═══════════════════════════════════════════════════════════════════════════════
#  Tree Viewer Window
# ═══════════════════════════════════════════════════════════════════════════════

class TreeViewerWindow(tk.Toplevel):
    """
    Second window — search tree visualizer.
    Created (or refreshed) by the main app when the user clicks
    "View Search Tree".
    """

    PLAY_INTERVAL_MS = 650   # ms between auto-steps

    def __init__(self, master, algo_name: str, start: int, requests: list):
        super().__init__(master)
        self.title(f"Search Tree — {algo_name}  |  start={start}  requests={sorted(requests)}")
        self.configure(bg=BG)
        self.geometry("1000x700")
        self.minsize(750, 500)
        self.resizable(True, True)

        self._algo_name = algo_name
        self._start     = start
        self._requests  = list(requests)
        self._step_idx  = 0
        self._play_job  = None

        # Build tree data
        algo_short = algo_name.split("(")[0].strip().replace("*", "star").replace(" ", "")
        # map display name -> short key used in generate_steps
        _map = {
            "BFS": "BFS", "DFS": "DFS", "UCS": "UCS", "A*": "A*",
        }
        short_key = algo_name.split("(")[0].strip()   # "BFS", "DFS", "UCS", "A*"

        self._nodes, self._edges, self._steps = generate_steps(
            short_key, start, requests)
        self._positions, self._canvas_h = compute_layout(self._nodes, self._edges)
        self._algo_color = ALGO_COLORS.get(algo_name, ACCENT)

        self._build_ui()
        self._render()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Top info bar
        info = tk.Frame(self, bg=PANEL, height=44)
        info.pack(fill="x")
        info.pack_propagate(False)
        tk.Label(info, text=f"Algorithm: {self._algo_name}",
                 font=("Courier", 11, "bold"), fg=self._algo_color, bg=PANEL
                 ).pack(side="left", padx=16, pady=10)
        tk.Label(info,
                 text=f"Start: {self._start}   Requests: {sorted(self._requests)}   "
                      f"States in tree: {len(self._nodes)}",
                 font=("Courier", 10), fg=DIM, bg=PANEL).pack(side="left", padx=8)

        # Canvas with scrollbars
        canvas_frame = tk.Frame(self, bg=BG)
        canvas_frame.pack(fill="both", expand=True, padx=8, pady=(6, 0))

        self._hbar = tk.Scrollbar(canvas_frame, orient="horizontal", bg=CARD)
        self._vbar = tk.Scrollbar(canvas_frame, orient="vertical",   bg=CARD)
        self._hbar.pack(side="bottom", fill="x")
        self._vbar.pack(side="right",  fill="y")

        self._canvas = tk.Canvas(
            canvas_frame, bg=BG, highlightthickness=0,
            xscrollcommand=self._hbar.set,
            yscrollcommand=self._vbar.set,
        )
        self._canvas.pack(fill="both", expand=True)
        self._hbar.config(command=self._canvas.xview)
        self._vbar.config(command=self._canvas.yview)
        self._canvas.config(scrollregion=(0, 0, CANVAS_W + 40, self._canvas_h + 40))

        # Message bar
        msg_frame = tk.Frame(self, bg=CARD, height=38)
        msg_frame.pack(fill="x")
        msg_frame.pack_propagate(False)
        self._msg_lbl = tk.Label(msg_frame, text="", font=("Courier", 10),
                                  fg=TEXT, bg=CARD, anchor="w", wraplength=900)
        self._msg_lbl.pack(side="left", padx=14, pady=8)

        # Controls bar
        ctrl = tk.Frame(self, bg=PANEL, height=52)
        ctrl.pack(fill="x")
        ctrl.pack_propagate(False)

        def cbtn(text, cmd, color=DIM):
            return tk.Button(ctrl, text=text, font=("Courier", 10),
                             fg=BG, bg=color, relief="flat", bd=0,
                             activebackground=color, cursor="hand2",
                             command=cmd, padx=12, pady=6)

        self._btn_prev = cbtn("◀  Prev", self._step_back)
        self._btn_prev.pack(side="left", padx=(14, 4), pady=10)

        self._btn_play = cbtn("▶  Play", self._toggle_play, self._algo_color)
        self._btn_play.pack(side="left", padx=4, pady=10)

        self._btn_next = cbtn("Next  ▶", self._step_fwd)
        self._btn_next.pack(side="left", padx=4, pady=10)

        cbtn("Reset", self._reset, WARN).pack(side="left", padx=(12, 4), pady=10)

        self._step_lbl = tk.Label(ctrl, text="", font=("Courier", 10),
                                   fg=DIM, bg=PANEL)
        self._step_lbl.pack(side="left", padx=12)

        # Legend
        legend_items = [
            (NODE_UNVISITED_BORDER, "Not reached"),
            (NODE_FRONTIER_BORDER,  "In frontier (queued)"),
            (NODE_EXPLORED_BORDER,  "Explored (popped)"),
            (self._algo_color,      "Current node"),
            (NODE_GOAL_BORDER,      "Goal reached"),
            (EDGE_PATH,             "Solution path"),
        ]
        for color, label in legend_items:
            dot = tk.Canvas(ctrl, width=12, height=12, bg=PANEL, highlightthickness=0)
            dot.create_oval(1, 1, 11, 11, fill=color, outline="")
            dot.pack(side="right", padx=(0, 2), pady=18)
            tk.Label(ctrl, text=label, font=("Courier", 9), fg=DIM, bg=PANEL
                     ).pack(side="right", padx=(8, 0), pady=18)

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _render(self):
        if not self._steps:
            return
        c     = self._canvas
        step  = self._steps[self._step_idx]
        c.delete("all")

        cur_key  = step["current_key"]
        frontier = step["frontier_keys"]
        explored = step["explored_keys"]
        goal_key = step["goal_key"]
        sol_path = step["solution_path"]   # list of floors in order

        # ── Edges ──
        for edge in self._edges:
            fk, tk_ = edge["from"], edge["to"]
            if fk not in self._positions or tk_ not in self._positions:
                continue
            x1, y1 = self._positions[fk]
            x2, y2 = self._positions[tk_]

            # Is this edge part of the solution path?
            in_path = False
            if len(sol_path) >= 2:
                for i in range(1, len(sol_path)):
                    pf = sol_path[i - 1]
                    cf = sol_path[i]
                    pk = _state_key(pf, sorted(
                        [f for f in self._requests if f not in sol_path[:i]]))
                    ck_ = _state_key(cf, sorted(
                        [f for f in self._requests if f not in sol_path[:i + 1]]))
                    if fk == pk and tk_ == ck_:
                        in_path = True
                        break

            color = EDGE_PATH    if in_path else EDGE_DEFAULT
            width = EDGE_PATH_W  if in_path else 1.0
            c.create_line(x1, y1, x2, y2, fill=color, width=width)

            # Edge cost label
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            c.create_text(mx + 6, my - 8, text=str(edge["cost"]),
                          font=("Courier", 9), fill="#666688")

        # ── Nodes ──
        for key, nd in self._nodes.items():
            if key not in self._positions:
                continue
            x, y = self._positions[key]

            # Determine visual state
            if key == goal_key:
                fill    = NODE_GOAL
                border  = NODE_GOAL_BORDER
                bwidth  = 2.5
                tfill   = TEXT_GOAL
            elif key == cur_key:
                fill    = "#1a2a4a"
                border  = self._algo_color
                bwidth  = 3.0
                tfill   = TEXT_CURRENT
            elif key in frontier:
                fill    = NODE_FRONTIER
                border  = NODE_FRONTIER_BORDER
                bwidth  = 1.5
                tfill   = TEXT_FRONTIER
            elif key in explored:
                fill    = NODE_EXPLORED
                border  = NODE_EXPLORED_BORDER
                bwidth  = 1.5
                tfill   = TEXT_EXPLORED
            else:
                fill    = NODE_UNVISITED
                border  = NODE_UNVISITED_BORDER
                bwidth  = 1.0
                tfill   = TEXT_UNVISITED

            c.create_oval(x - NODE_R, y - NODE_R, x + NODE_R, y + NODE_R,
                          fill=fill, outline=border, width=bwidth)

            # Floor number (big)
            c.create_text(x, y - 7, text=f"F{nd['floor']}",
                          font=("Courier", 10, "bold"), fill=tfill)
            # Remaining requests (small)
            rem_str = ",".join(str(f) for f in sorted(nd["remaining"])) or "done"
            c.create_text(x, y + 8, text=f"[{rem_str}]",
                          font=("Courier", 8), fill=tfill)

        # ── Update labels ──
        self._msg_lbl.config(text=step["message"])
        total = len(self._steps)
        self._step_lbl.config(
            text=f"Step {self._step_idx + 1} / {total}")
        self._btn_prev.config(state="normal" if self._step_idx > 0 else "disabled")
        self._btn_next.config(state="normal" if self._step_idx < total - 1 else "disabled")

    # ── Controls ─────────────────────────────────────────────────────────────

    def _step_fwd(self):
        if self._step_idx < len(self._steps) - 1:
            self._step_idx += 1
            self._render()

    def _step_back(self):
        if self._step_idx > 0:
            self._step_idx -= 1
            self._render()

    def _reset(self):
        self._stop_play()
        self._step_idx = 0
        self._render()

    def _toggle_play(self):
        if self._play_job:
            self._stop_play()
        else:
            self._btn_play.config(text="⏸  Pause")
            self._auto_step()

    def _auto_step(self):
        if self._step_idx < len(self._steps) - 1:
            self._step_idx += 1
            self._render()
            self._play_job = self.after(self.PLAY_INTERVAL_MS, self._auto_step)
        else:
            self._stop_play()

    def _stop_play(self):
        if self._play_job:
            self.after_cancel(self._play_job)
            self._play_job = None
        self._btn_play.config(text="▶  Play")

    def destroy(self):
        self._stop_play()
        super().destroy()


# ═══════════════════════════════════════════════════════════════════════════════
#  Main Application  (single elevator, unchanged core + tree viewer button)
# ═══════════════════════════════════════════════════════════════════════════════

class ElevatorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Elevator Optimization System")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.geometry("1100x750")
        self.minsize(900, 600)

        self._animation_running = False
        self._stop_animation    = False
        self._results           = {}
        self._tree_window       = None   # reference to open TreeViewerWindow

        self._build_ui()
        self._update_building_canvas()

    # ── UI Construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        title_bar = tk.Frame(self, bg=PANEL, height=54)
        title_bar.pack(fill="x", side="top")
        title_bar.pack_propagate(False)
        tk.Label(title_bar, text="⬡  ELEVATOR OPTIMIZER",
                 font=("Courier", 16, "bold"), fg=ACCENT, bg=PANEL
                 ).pack(side="left", padx=20, pady=14)
        tk.Label(title_bar, text="BFS · DFS · UCS · A*",
                 font=("Courier", 11), fg=DIM, bg=PANEL).pack(side="right", padx=20)

        main = tk.Frame(self, bg=BG)
        main.pack(fill="both", expand=True, padx=12, pady=10)

        self._build_left_panel(main)
        self._build_building_canvas(main)
        self._build_right_panel(main)

    def _build_left_panel(self, parent):
        frame = tk.Frame(parent, bg=PANEL, width=240)
        frame.pack(side="left", fill="y", padx=(0, 10))
        frame.pack_propagate(False)

        def section(label):
            tk.Label(frame, text=label, font=("Courier", 9, "bold"),
                     fg=ACCENT, bg=PANEL).pack(anchor="w", padx=16, pady=(18, 4))
            tk.Frame(frame, bg=ACCENT, height=1).pack(fill="x", padx=16)

        section("BUILDING")
        floors_row = tk.Frame(frame, bg=PANEL)
        floors_row.pack(fill="x", padx=16, pady=(8, 0))
        tk.Label(floors_row, text="Total floors", font=("Courier", 10),
                 fg=DIM, bg=PANEL).pack(side="left")
        self.floors_var = tk.IntVar(value=10)
        sb = tk.Spinbox(floors_row, from_=3, to=25, textvariable=self.floors_var,
                        width=4, font=("Courier", 11), bg=CARD, fg=TEXT,
                        buttonbackground=CARD, relief="flat",
                        command=self._on_floors_changed)
        sb.pack(side="right")
        sb.bind("<Return>", lambda e: self._on_floors_changed())

        start_row = tk.Frame(frame, bg=PANEL)
        start_row.pack(fill="x", padx=16, pady=6)
        tk.Label(start_row, text="Start floor", font=("Courier", 10),
                 fg=DIM, bg=PANEL).pack(side="left")
        self.start_var = tk.IntVar(value=1)
        self.start_spin = tk.Spinbox(start_row, from_=1, to=25,
                                     textvariable=self.start_var,
                                     width=4, font=("Courier", 11),
                                     bg=CARD, fg=ACCENT,
                                     buttonbackground=CARD, relief="flat")
        self.start_spin.pack(side="right")

        section("REQUESTS")
        tk.Label(frame, text="Click floors on the building\nor type below (comma-sep):",
                 font=("Courier", 9), fg=DIM, bg=PANEL, justify="left"
                 ).pack(anchor="w", padx=16, pady=(8, 4))
        self.req_entry = tk.Entry(frame, font=("Courier", 12), bg=CARD, fg=ACCENT2,
                                  insertbackground=ACCENT2, relief="flat", bd=6)
        self.req_entry.pack(fill="x", padx=16, pady=(0, 6))
        self.req_entry.bind("<Return>", lambda e: self._parse_entry_requests())
        btn_row = tk.Frame(frame, bg=PANEL)
        btn_row.pack(fill="x", padx=16)
        self._btn(btn_row, "Set",   self._parse_entry_requests, ACCENT ).pack(side="left", padx=(0, 6))
        self._btn(btn_row, "Clear", self._clear_requests,        DANGER ).pack(side="left")

        section("ALGORITHM")
        self.algo_var = tk.StringVar(value=list(ALGORITHMS.keys())[3])
        for name in ALGORITHMS:
            color = ALGO_COLORS[name]
            rb = tk.Radiobutton(frame, text=name, variable=self.algo_var, value=name,
                                font=("Courier", 10), fg=color, bg=PANEL,
                                selectcolor=CARD, activebackground=PANEL,
                                activeforeground=color, command=self._on_algo_changed)
            rb.pack(anchor="w", padx=16, pady=2)
        self.algo_info_lbl = tk.Label(frame, text=ALGO_INFO[self.algo_var.get()],
                                      font=("Courier", 9), fg=DIM, bg=PANEL,
                                      wraplength=210, justify="left")
        self.algo_info_lbl.pack(anchor="w", padx=16, pady=(4, 0))

        section("ANIMATION SPEED")
        speed_row = tk.Frame(frame, bg=PANEL)
        speed_row.pack(fill="x", padx=16, pady=(8, 0))
        tk.Label(speed_row, text="Slow", font=("Courier", 9), fg=DIM, bg=PANEL).pack(side="left")
        tk.Label(speed_row, text="Fast", font=("Courier", 9), fg=DIM, bg=PANEL).pack(side="right")
        self.speed_var = tk.DoubleVar(value=0.35)
        tk.Scale(frame, from_=0.8, to=0.05, resolution=0.05,
                 variable=self.speed_var, orient="horizontal",
                 bg=PANEL, fg=TEXT, troughcolor=CARD, highlightthickness=0,
                 showvalue=False).pack(fill="x", padx=16)

        tk.Frame(frame, bg=PANEL).pack(expand=True)

        self.run_btn = tk.Button(frame, text="▶  RUN",
                                 font=("Courier", 13, "bold"),
                                 fg=BG, bg=ACCENT, relief="flat", bd=0,
                                 activebackground=ACCENT2, activeforeground=BG,
                                 cursor="hand2", command=self._run)
        self.run_btn.pack(fill="x", padx=16, pady=(0, 6), ipady=10)

        self.stop_btn = tk.Button(frame, text="■  STOP", font=("Courier", 11),
                                  fg=BG, bg=DANGER, relief="flat", bd=0,
                                  activebackground="#c04040", activeforeground=BG,
                                  cursor="hand2", command=self._stop,
                                  state="disabled")
        self.stop_btn.pack(fill="x", padx=16, pady=(0, 6), ipady=6)

        # ── View Search Tree button ──────────────────────────────────────────
        self.tree_btn = tk.Button(
            frame, text="⬡  View Search Tree",
            font=("Courier", 10, "bold"),
            fg=BG, bg="#534AB7", relief="flat", bd=0,
            activebackground="#7F77DD", activeforeground=BG,
            cursor="hand2", command=self._open_tree_viewer)
        self.tree_btn.pack(fill="x", padx=16, pady=(0, 16), ipady=8)

    def _build_building_canvas(self, parent):
        wrapper = tk.Frame(parent, bg=PANEL)
        wrapper.pack(side="left", fill="both", expand=True, padx=(0, 10))
        tk.Label(wrapper, text="BUILDING", font=("Courier", 9, "bold"),
                 fg=ACCENT, bg=PANEL).pack(anchor="w", padx=14, pady=(12, 2))
        tk.Frame(wrapper, bg=ACCENT, height=1).pack(fill="x", padx=14)
        tk.Label(wrapper, text="Click any floor to toggle a request",
                 font=("Courier", 9), fg=DIM, bg=PANEL
                 ).pack(anchor="w", padx=14, pady=(4, 8))

        self.canvas_frame = tk.Frame(wrapper, bg=PANEL)
        self.canvas_frame.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        self.canvas = tk.Canvas(self.canvas_frame, bg=PANEL,
                                highlightthickness=0, cursor="hand2")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self._update_building_canvas())
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        self._requests      = set()
        self._elevator_floor = None
        self._visited       = set()

    def _build_right_panel(self, parent):
        frame = tk.Frame(parent, bg=PANEL, width=260)
        frame.pack(side="right", fill="y")
        frame.pack_propagate(False)

        def section(label):
            tk.Label(frame, text=label, font=("Courier", 9, "bold"),
                     fg=ACCENT, bg=PANEL).pack(anchor="w", padx=16, pady=(18, 4))
            tk.Frame(frame, bg=ACCENT, height=1).pack(fill="x", padx=16)

        section("RESULTS")
        self.results_frame = tk.Frame(frame, bg=PANEL)
        self.results_frame.pack(fill="x", padx=16, pady=8)
        self._render_results_placeholder()

        section("PATH")
        self.path_lbl = tk.Label(frame, text="—", font=("Courier", 10),
                                  fg=DIM, bg=PANEL, wraplength=228, justify="left")
        self.path_lbl.pack(anchor="w", padx=16, pady=(8, 0))

        section("STATUS")
        self.status_lbl = tk.Label(frame, text="Configure and press RUN",
                                    font=("Courier", 10), fg=DIM, bg=PANEL,
                                    wraplength=228, justify="left")
        self.status_lbl.pack(anchor="w", padx=16, pady=(8, 0))

        section("COMPARE ALL")
        tk.Button(frame, text="⇄  Compare All Algorithms",
                  font=("Courier", 10), fg=BG, bg=WARN, relief="flat", bd=0,
                  cursor="hand2", activebackground="#e09030", activeforeground=BG,
                  command=self._compare_all
                  ).pack(fill="x", padx=16, pady=(8, 0), ipady=6)

        section("LEGEND")
        for sym, col, desc in [
            ("■", ACCENT,  "Elevator (current)"),
            ("●", ACCENT2, "Served floor"),
            ("○", WARN,    "Pending request"),
            ("▷", "#aaa",  "Start floor"),
        ]:
            row = tk.Frame(frame, bg=PANEL)
            row.pack(anchor="w", padx=16, pady=2)
            tk.Label(row, text=sym, font=("Courier", 12), fg=col,
                     bg=PANEL, width=2).pack(side="left")
            tk.Label(row, text=desc, font=("Courier", 10), fg=DIM,
                     bg=PANEL).pack(side="left", padx=4)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _btn(self, parent, text, cmd, color):
        return tk.Button(parent, text=text, font=("Courier", 10),
                         fg=BG, bg=color, relief="flat", bd=0,
                         activebackground=color, activeforeground=BG,
                         cursor="hand2", command=cmd, padx=8, pady=4)

    def _on_algo_changed(self):
        self.algo_info_lbl.config(text=ALGO_INFO[self.algo_var.get()])

    def _on_floors_changed(self):
        n = self.floors_var.get()
        self.start_spin.config(to=n)
        if self.start_var.get() > n:
            self.start_var.set(n)
        self._requests      = {r for r in self._requests if 1 <= r <= n}
        self._elevator_floor = None
        self._visited       = set()
        self._update_building_canvas()

    # ── Request Management ────────────────────────────────────────────────────

    def _parse_entry_requests(self):
        raw = self.req_entry.get()
        try:
            floors  = [int(x.strip()) for x in raw.split(",") if x.strip()]
            n       = self.floors_var.get()
            invalid = [f for f in floors if not (1 <= f <= n)]
            if invalid:
                messagebox.showerror("Invalid floors",
                                     f"Floors {invalid} are out of range (1–{n}).")
                return
            self._requests      = set(floors)
            self._elevator_floor = None
            self._visited       = set()
            self._update_building_canvas()
        except ValueError:
            messagebox.showerror("Error", "Please enter comma-separated numbers.")

    def _clear_requests(self):
        self._requests.clear()
        self._elevator_floor = None
        self._visited        = set()
        self.req_entry.delete(0, tk.END)
        self._update_building_canvas()

    def _sync_entry(self):
        self.req_entry.delete(0, tk.END)
        self.req_entry.insert(0, ", ".join(str(f) for f in sorted(self._requests)))

    # ── Building Canvas ───────────────────────────────────────────────────────

    def _update_building_canvas(self, elevator_floor=None, visited=None):
        if elevator_floor is not None:
            self._elevator_floor = elevator_floor
        if visited is not None:
            self._visited = visited

        self.canvas.delete("all")
        n     = self.floors_var.get()
        start = self.start_var.get()
        w     = self.canvas.winfo_width()  or 320
        h     = self.canvas.winfo_height() or 500

        shaft_x1, shaft_x2 = w // 2 - 28, w // 2 + 28
        total_h = n * FLOOR_H
        top_y   = max(10, (h - total_h) // 2)

        self.canvas.create_rectangle(shaft_x1, top_y, shaft_x2, top_y + total_h,
                                     fill=SHAFT, outline=DIM, width=1)

        for i, floor in enumerate(range(n, 0, -1)):
            y1 = top_y + i * FLOOR_H
            y2 = y1 + FLOOR_H
            cy = (y1 + y2) // 2

            self.canvas.create_line(0, y2, w, y2, fill="#2a2f45", width=1)
            self.canvas.create_text(shaft_x1 - 12, cy, text=str(floor),
                                    font=("Courier", 10), fill=DIM, anchor="e")

            if floor == start and self._elevator_floor is None:
                self.canvas.create_text(shaft_x2 + 12, cy, text="▷",
                                        font=("Courier", 11), fill="#aaaaaa", anchor="w")

            if floor in self._visited:
                self.canvas.create_text(shaft_x2 + 12, cy, text="●",
                                        font=("Courier", 13, "bold"),
                                        fill=ACCENT2, anchor="w")
            elif floor in self._requests:
                self.canvas.create_text(shaft_x2 + 12, cy, text="○",
                                        font=("Courier", 13), fill=WARN, anchor="w")

            if floor == self._elevator_floor:
                self.canvas.create_rectangle(shaft_x1 + 4, y1 + 4,
                                             shaft_x2 - 4, y2 - 4,
                                             fill=ACCENT, outline="", width=0)
                self.canvas.create_text((shaft_x1 + shaft_x2) // 2, cy,
                                        text="■", font=("Courier", 14, "bold"), fill=BG)

        self._floor_hitboxes = []
        for i, floor in enumerate(range(n, 0, -1)):
            y1 = top_y + i * FLOOR_H
            y2 = y1 + FLOOR_H
            self._floor_hitboxes.append((floor, y1, y2))

    def _on_canvas_click(self, event):
        if self._animation_running:
            return
        for floor, y1, y2 in getattr(self, "_floor_hitboxes", []):
            if y1 <= event.y <= y2:
                if floor in self._requests:
                    self._requests.discard(floor)
                else:
                    self._requests.add(floor)
                self._sync_entry()
                self._elevator_floor = None
                self._visited        = set()
                self._update_building_canvas()
                return

    # ── Results Panel ─────────────────────────────────────────────────────────

    def _render_results_placeholder(self):
        for w in self.results_frame.winfo_children():
            w.destroy()
        tk.Label(self.results_frame, text="No results yet.",
                 font=("Courier", 10), fg=DIM, bg=PANEL).pack(anchor="w")

    def _render_results(self, results: dict, highlight: str = None):
        for w in self.results_frame.winfo_children():
            w.destroy()
        for c, h in zip([0, 1, 2], ["Algorithm", "Cost", "Nodes"]):
            tk.Label(self.results_frame, text=h, font=("Courier", 9, "bold"),
                     fg=DIM, bg=PANEL).grid(row=0, column=c, sticky="w", padx=(0, 12))
        tk.Frame(self.results_frame, bg=DIM, height=1).grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=2)

        best_cost = min(r["cost"] for r in results.values()) if results else None
        for row, (name, res) in enumerate(results.items(), start=2):
            short      = name.split("(")[0].strip()
            color      = ALGO_COLORS[name]
            bg         = CARD if name == highlight else PANEL
            cost_color = ACCENT2 if res["cost"] == best_cost else TEXT
            for col, val in enumerate([short, str(res["cost"]), str(res["nodes_explored"])]):
                tk.Label(self.results_frame, text=val, font=("Courier", 10),
                         fg=color if col == 0 else (cost_color if col == 1 else TEXT),
                         bg=bg, anchor="w"
                         ).grid(row=row, column=col, sticky="w", padx=(0, 12), pady=1)

    # ── Run & Animate ─────────────────────────────────────────────────────────

    def _validate_inputs(self):
        n     = self.floors_var.get()
        start = self.start_var.get()
        if not (1 <= start <= n):
            messagebox.showerror("Error", f"Start floor must be between 1 and {n}.")
            return False
        if not self._requests:
            messagebox.showerror("Error",
                                 "Please add at least one request floor.")
            return False
        return True

    def _run(self):
        if not self._validate_inputs() or self._animation_running:
            return
        algo_name = self.algo_var.get()
        start     = self.start_var.get()
        requests  = list(self._requests)
        result    = ALGORITHMS[algo_name](start, requests)

        self._results[algo_name] = result
        self._render_results({algo_name: result}, highlight=algo_name)
        self.path_lbl.config(
            text=" → ".join(str(f) for f in [start] + result["path"]),
            fg=ALGO_COLORS[algo_name])
        self.status_lbl.config(text="Animating…", fg=DIM)

        self._animation_running = True
        self._stop_animation    = False
        self.run_btn.config(state="disabled")
        self.stop_btn.config(state="normal")

        threading.Thread(target=self._animate,
                         args=(start, result["path"], algo_name),
                         daemon=True).start()

    def _animate(self, start: int, path: list, algo_name: str):
        delay   = self.speed_var.get()
        visited = set()
        self._update_building_canvas(elevator_floor=start, visited=visited)
        prev = start
        for target in path:
            if self._stop_animation:
                break
            direction = 1 if target > prev else -1
            for f in range(prev + direction, target + direction, direction):
                if self._stop_animation:
                    break
                self.canvas.after(0, self._update_building_canvas, f, set(visited))
                time.sleep(delay)
            if not self._stop_animation:
                visited.add(target)
                self.canvas.after(0, self._update_building_canvas, target, set(visited))
                self.canvas.after(0, self.status_lbl.config,
                                  {"text": f"Serving floor {target}…", "fg": ACCENT2})
                time.sleep(delay * 1.5)
            prev = target

        if not self._stop_animation:
            self.canvas.after(0, self.status_lbl.config,
                              {"text": f"Done! Cost: {path_total_cost(start, path)} floors",
                               "fg": ACCENT2})
        else:
            self.canvas.after(0, self.status_lbl.config,
                              {"text": "Animation stopped.", "fg": WARN})
        self.canvas.after(0, self._animation_done)

    def _animation_done(self):
        self._animation_running = False
        self.run_btn.config(state="normal")
        self.stop_btn.config(state="disabled")

    def _stop(self):
        self._stop_animation = True

    # ── Compare All ───────────────────────────────────────────────────────────

    def _compare_all(self):
        if not self._validate_inputs():
            return
        start    = self.start_var.get()
        requests = list(self._requests)
        results  = {name: fn(start, requests) for name, fn in ALGORITHMS.items()}
        self._results = results
        self._render_results(results)
        best = min(results, key=lambda k: results[k]["cost"])
        self.path_lbl.config(
            text=f"Best: {best.split('(')[0].strip()}\n" +
                 " → ".join(str(f) for f in [start] + results[best]["path"]),
            fg=ALGO_COLORS[best])
        self.status_lbl.config(
            text=f"A* used fewest nodes ({results[list(ALGORITHMS.keys())[3]]['nodes_explored']}).",
            fg=DIM)
        self._elevator_floor = None
        self._visited        = set()
        self._update_building_canvas()

    # ── Tree Viewer ───────────────────────────────────────────────────────────

    def _open_tree_viewer(self):
        if not self._validate_inputs():
            return

        # Close previous viewer if still open
        if self._tree_window and self._tree_window.winfo_exists():
            self._tree_window.destroy()

        algo_name = self.algo_var.get()
        start     = self.start_var.get()
        requests  = list(self._requests)

        if len(requests) > 4:
            messagebox.showinfo(
                "Tree Viewer",
                "The tree viewer shows up to 4 request floors to keep the diagram readable.\n"
                f"Showing first 4 of your {len(requests)} requests: {sorted(requests)[:4]}")
            requests = sorted(requests)[:4]

        self._tree_window = TreeViewerWindow(self, algo_name, start, requests)
        self._tree_window.focus()


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = ElevatorApp()
    app.mainloop()