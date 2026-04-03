############################################################
# THIS GUI IS AI-GENERATED WITH MANUAL OUTPUT VERIFICATION #
############################################################

import tkinter as tk
from tkinter import ttk, filedialog
import inspect
import ast
import re
import traceback
import os
import json
import numpy as np
from dataclasses import dataclass, field

try:
    from create_sim import SimBase
    IMPORT_ERROR = None
except Exception as e:
    SimBase = None
    IMPORT_ERROR = str(e)

try:
    from Basilisk.utilities.MonteCarlo.Dispersions import (
        InertiaTensorDispersion,
        UniformEulerAngleMRPDispersion,
        NormalVectorCartDispersion,
    )
    from create_sim import MonteCarlo
    _MC_IMPORT_ERROR = None
except Exception as e:
    _MC_IMPORT_ERROR = str(e)
    InertiaTensorDispersion = UniformEulerAngleMRPDispersion = NormalVectorCartDispersion = None
    MonteCarlo = None

# Registry of scObject.hub attributes that can be dispersed.
# Each entry: (attr_name, basilisk_path, disp_class_fn, default_bounds, param_specs, description)
# param_specs is a list of (label, unit_hint) for each extra bound argument.
_HUB_ATTR_REGISTRY = [
    (
        "sigma_BNInit",
        "scObject.hub.sigma_BNInit",
        lambda: UniformEulerAngleMRPDispersion,
        [],
        [],
        "Initial attitude (MRP)",
    ),
    (
        "omega_BN_BInit",
        "scObject.hub.omega_BN_BInit",
        lambda: NormalVectorCartDispersion,
        ["0.0", f"{np.pi / 4:.4f}"],
        [("Mean", "rad/s"), ("Std Dev", "rad/s")],
        "Initial angular velocity",
    ),
    (
        "IHubPntBc_B",
        "scObject.hub.IHubPntBc_B",
        lambda: InertiaTensorDispersion,
        ["0.05"],
        [("Std Dev", "fraction")],
        "Inertia tensor",
    ),
    (
        "r_BcB_B",
        "scObject.hub.r_BcB_B",
        lambda: NormalVectorCartDispersion,
        ["0.0", "0.01"],
        [("Mean", "m"), ("Std Dev", "m")],
        "Center-of-mass offset",
    ),
]


# ---------------------------------------------------------------------------
# Data structures for the Messages tab
# ---------------------------------------------------------------------------

@dataclass
class Port:
    attr_name: str
    label: str
    oval_id: int
    cx: float
    cy: float
    msg_type: str = ""   # Basilisk payload type name for compatibility checking


@dataclass
class Block:
    name: str
    obj: object
    out_ports: list   # list[Port]
    in_ports: list    # list[Port]
    item_ids: list    # all canvas item IDs in this block
    x: float
    y: float
    w: float
    h: float


@dataclass
class Connection:
    src_block: str
    src_port: str     # attr_name on src block obj; '__self__' for standalone OutMsg
    dst_block: str
    dst_port: str
    line_id: int
    applied: bool = False


@dataclass
class RecEntry:
    block_name: str
    port_attr: str    # attr_name on block.obj; '__self__' for standalone
    label: str        # human-readable "block · port" label
    ring_id: int      # canvas oval ID of halo ring; -1 if not currently drawn
    rec_obj: object   # Basilisk recorder object (None until apply_recorders)


# ---------------------------------------------------------------------------
# Tooltip helper
# ---------------------------------------------------------------------------

class ToolTip:
    """Show a tooltip near a widget on hover."""

    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tip_window = None
        widget.bind("<Enter>", self._show)
        widget.bind("<Leave>", self._hide)

    def _show(self, event=None):
        if self.tip_window or not self.text:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 4
        self.tip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(
            tw, text=self.text, justify=tk.LEFT,
            background="#ffffe0", relief=tk.SOLID, borderwidth=1,
            wraplength=380, font=("TkDefaultFont", 9),
        )
        label.pack(ipadx=4, ipady=2)

    def _hide(self, event=None):
        if self.tip_window:
            self.tip_window.destroy()
            self.tip_window = None


# ---------------------------------------------------------------------------
# Messages Tab
# ---------------------------------------------------------------------------

class MessagesTab:
    BLOCK_W = 220
    PORT_R = 7
    PORT_SPACING = 26
    TITLE_H = 30
    PAD = 10
    SNAP_R = 16   # snap radius for inport hit-test

    def __init__(self, parent_frame, sim_gui):
        self._sim_gui = sim_gui
        self._blocks: dict[str, Block] = {}
        self._connections: list[Connection] = []
        self._drag_block = None    # block name being dragged
        self._drag_prev = None     # (x, y) of last drag event
        self._conn_src = None      # (block_name, attr_name, cx, cy) of connection source
        self._rubber_line = None   # canvas id of temporary rubber-band line

        # Recording state
        self._recorders: dict[tuple, RecEntry] = {}          # (block, port) -> RecEntry
        self._rec_panel_rows: dict[tuple, tuple] = {}        # (block, port) -> (row_frame, dot_label)
        self._save_dir_var: tk.StringVar | None = None       # set in _build
        self._rec_list_frame: ttk.Frame | None = None        # set in _build

        self._build(parent_frame)

    def _build(self, parent):
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill=tk.X, padx=6, pady=(6, 2))

        ttk.Button(toolbar, text="Refresh", command=self.refresh).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(toolbar, text="Apply Connections", command=self._apply_connections).pack(side=tk.LEFT)
        ttk.Label(
            toolbar,
            text="  Drag \u25cf green\u2192gray to connect.  Right-click connection to remove.  Shift+click output port to toggle recording.",
            foreground="#555555", font=("TkDefaultFont", 8),
        ).pack(side=tk.LEFT, padx=8)

        # Content area: canvas (left) + recording panel (right)
        content = ttk.Frame(parent)
        content.pack(fill=tk.BOTH, expand=True)

        # ---- Recording panel (right) — pack BEFORE canvas so it reserves space first ----
        rec_panel = ttk.LabelFrame(content, text="Recording", padding=(10, 8))
        rec_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(4, 6), pady=6)

        # Save directory row
        dir_row = ttk.Frame(rec_panel)
        dir_row.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(dir_row, text="Save dir:", anchor="w").pack(side=tk.LEFT)
        self._save_dir_var = tk.StringVar(value="./rec_output")
        dir_entry = ttk.Entry(dir_row, textvariable=self._save_dir_var, width=16)
        dir_entry.pack(side=tk.LEFT, padx=(4, 2))
        browse_btn = ttk.Button(dir_row, text="\u2026", width=2, command=self._browse_save_dir)
        browse_btn.pack(side=tk.LEFT)
        ToolTip(browse_btn, "Choose a folder where recorded .npz files will be saved.")

        ttk.Separator(rec_panel, orient="horizontal").pack(fill=tk.X, pady=(4, 6))

        # Legend
        legend = ttk.Frame(rec_panel)
        legend.pack(anchor="w", pady=(0, 4))
        tk.Label(legend, text="\u25cf", foreground="#e65c00",
                 font=("TkDefaultFont", 10)).pack(side=tk.LEFT)
        ttk.Label(legend, text=" armed  ", foreground="#555555",
                  font=("TkDefaultFont", 8)).pack(side=tk.LEFT)
        tk.Label(legend, text="\u25cf", foreground="#1a7a1a",
                 font=("TkDefaultFont", 10)).pack(side=tk.LEFT)
        ttk.Label(legend, text=" recorded", foreground="#555555",
                  font=("TkDefaultFont", 8)).pack(side=tk.LEFT)

        ttk.Label(rec_panel, text="Armed recorders:",
                  foreground="#555555", font=("TkDefaultFont", 8)).pack(anchor="w")

        self._rec_list_frame = ttk.Frame(rec_panel)
        self._rec_list_frame.pack(fill=tk.X, pady=(2, 0))

        ttk.Separator(rec_panel, orient="horizontal").pack(fill=tk.X, pady=(8, 4))

        apply_rec_btn = ttk.Button(
            rec_panel, text="Apply Recorders",
            command=self._apply_recorders,
        )
        apply_rec_btn.pack(fill=tk.X, pady=(2, 0))
        ToolTip(apply_rec_btn,
                "Attach recorders to the simulation now.\n"
                "This is also done automatically before run_sim.")
        ttk.Label(rec_panel, text="(auto-applied on run_sim)",
                  foreground="#888888", font=("TkDefaultFont", 7)).pack(anchor="w")

        # ---- Canvas (left) ----
        self._canvas = tk.Canvas(content, bg="#e8e8e8", highlightthickness=0)
        self._canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self._canvas.bind("<Button-3>", self._on_right_click)

        self._canvas.create_text(
            20, 20, anchor="nw",
            text='Apply "__init__" in the Configure tab, then click Refresh.',
            fill="#888888", font=("TkDefaultFont", 11),
        )

    # ------------------------------------------------------------------
    # Module & port discovery
    # ------------------------------------------------------------------

    @staticmethod
    def _get_msg_type(msg) -> str:
        """Extract a canonical payload-type string from a Basilisk message object.

        Strategy:
        1. Prefer the last component of the module path when it ends with
           "Payload" (e.g. ``Basilisk.architecture.messaging.AttGuidMsgPayload``).
        2. Fall back to the class name with common SWIG suffixes stripped
           (``_C``, ``Reader``, ``Payload``).  Stripping "Payload" here lets
           older message types that embed it in the class name still match their
           reader counterparts.
        """
        try:
            mod = type(msg).__module__
            parts = mod.split(".")
            last = parts[-1]
            if last.endswith("Payload"):
                # Strip trailing "Payload" so both OutMsg and Reader collapse to
                # the same root name (e.g. "AttGuidMsg").
                return last[: -len("Payload")]
            # Fallback: normalise class name
            name = type(msg).__name__
            for suffix in ("_C", "Reader", "Payload"):
                if name.endswith(suffix):
                    name = name[: -len(suffix)]
                    break
            return name
        except Exception:
            return ""

    def _discover_modules(self, sim):
        """
        Scan sim.__dict__ for Basilisk objects that expose message ports.
        Returns dict: attr_name -> (obj, out_ports_list, in_ports_list)
        where each port list contains (attr_name, label, msg_type) tuples.
        '__self__' is used as attr_name for standalone OutMsg objects.
        """
        found = {}
        primitives = (int, float, str, list, dict, tuple, bool, bytes, type(None))

        for attr_name, obj in sim.__dict__.items():
            if obj is None or isinstance(obj, primitives):
                continue

            out_ports = []
            in_ports = []

            for a in dir(obj):
                if "Msg" not in a or a.startswith("_"):
                    continue
                try:
                    sub = getattr(obj, a)
                    mtype = self._get_msg_type(sub)
                    # Check OUTPUT first: OutMsg objects have write+addSubscriber.
                    # In newer Basilisk, OutMsg objects may also have subscribeTo
                    # (for output-chaining), so we must NOT rely on subscribeTo
                    # alone to distinguish direction.
                    if hasattr(sub, "write") and hasattr(sub, "addSubscriber"):
                        label = re.sub(r"OutMsg$|Out_Msg$|_OutMsg$", "", a).rstrip("_")
                        out_ports.append((a, label, mtype))
                    elif hasattr(sub, "subscribeTo"):
                        label = re.sub(r"InMsg$|In_Msg$|_InMsg$", "", a).rstrip("_")
                        in_ports.append((a, label, mtype))
                except Exception:
                    pass

            if out_ports or in_ports:
                found[attr_name] = (obj, out_ports, in_ports)
            elif hasattr(obj, "write") and hasattr(obj, "addSubscriber"):
                # Standalone output message (e.g. rwConfigMsg)
                mtype = self._get_msg_type(obj)
                found[attr_name] = (obj, [("__self__", attr_name, mtype)], [])

        return found

    # ------------------------------------------------------------------
    # Refresh / rebuild
    # ------------------------------------------------------------------

    def refresh(self):
        # Save current connections to restore after rebuild
        saved = [
            (c.src_block, c.src_port, c.dst_block, c.dst_port, c.applied)
            for c in self._connections
        ]

        self._canvas.delete("all")
        self._blocks = {}
        self._connections = []
        self._drag_block = None
        self._drag_prev = None
        self._conn_src = None
        self._rubber_line = None

        # Canvas was wiped — all ring canvas IDs are now invalid
        for entry in self._recorders.values():
            entry.ring_id = -1

        sim = self._sim_gui.sim
        if sim is None:
            self._canvas.create_text(
                20, 20, anchor="nw",
                text='Apply "__init__" in the Configure tab, then click Refresh.',
                fill="#888888", font=("TkDefaultFont", 11),
            )
            return

        found = self._discover_modules(sim)
        if not found:
            self._canvas.create_text(
                20, 20, anchor="nw",
                text=(
                    "No message-port modules found on the SimBase instance.\n\n"
                    "Apply build_spacecraft and/or add_rw_wheels in the Configure tab first,\n"
                    "then switch back to this tab (it refreshes automatically)."
                ),
                fill="#888888", font=("TkDefaultFont", 11),
            )
            return

        # Auto-layout: objects with more outputs go left, others go right
        left_items = [(n, d) for n, d in found.items() if len(d[1]) >= len(d[2])]
        right_items = [(n, d) for n, d in found.items() if len(d[1]) < len(d[2])]
        if not right_items:
            left_items = list(found.items())

        x_left, x_right = 40, 320
        y_l, y_r = 50, 50

        for name, (obj, out_ports, in_ports) in left_items:
            block = self._draw_block(name, obj, out_ports, in_ports, x_left, y_l)
            self._blocks[name] = block
            y_l += block.h + 28

        for name, (obj, out_ports, in_ports) in right_items:
            block = self._draw_block(name, obj, out_ports, in_ports, x_right, y_r)
            self._blocks[name] = block
            y_r += block.h + 28

        self._sim_gui._set_status("refresh", True,
                                   f" — {len(self._blocks)} module block(s) found")

        # Restore connections that still have valid port references
        for src_b, src_p, dst_b, dst_p, applied in saved:
            if src_b not in self._blocks or dst_b not in self._blocks:
                continue
            sp = self._get_port(src_b, "out", src_p)
            dp = self._get_port(dst_b, "in", dst_p)
            if sp and dp:
                lid = self._draw_conn_line(sp.cx, sp.cy, dp.cx, dp.cy, applied)
                self._connections.append(Connection(src_b, src_p, dst_b, dst_p, lid, applied))

        self._canvas.tag_lower("connection")

        # Restore halo rings for armed recorders that still have valid ports
        for key in list(self._recorders.keys()):
            entry = self._recorders[key]
            port = self._get_port(entry.block_name, "out", entry.port_attr)
            if port:
                recorded = entry.rec_obj is not None
                ring_id = self._draw_recorder_ring(
                    port.cx, port.cy, entry.block_name, entry.port_attr, recorded=recorded,
                )
                entry.ring_id = ring_id
            else:
                # Block/port no longer present — remove stale entry
                self._recorders.pop(key)
                self._remove_rec_panel_row(key)

    # ------------------------------------------------------------------
    # Block rendering
    # ------------------------------------------------------------------

    def _block_height(self, out_ports, in_ports):
        n = max(len(out_ports), len(in_ports), 1)
        return self.TITLE_H + n * self.PORT_SPACING + self.PAD

    def _draw_block(self, name, obj, out_ports, in_ports, x, y):
        w = self.BLOCK_W
        h = self._block_height(out_ports, in_ports)
        c = self._canvas
        tag = f"block:{name}"

        item_ids = []

        rect = c.create_rectangle(
            x, y, x + w, y + h,
            fill="#f5f5f5", outline="#666666", width=2, tags=tag,
        )
        item_ids.append(rect)

        sep = c.create_line(x, y + self.TITLE_H, x + w, y + self.TITLE_H, fill="#cccccc", tags=tag)
        item_ids.append(sep)

        title = c.create_text(
            x + w // 2, y + self.TITLE_H // 2,
            text=name, font=("TkDefaultFont", 10, "bold"), tags=tag,
        )
        item_ids.append(title)

        built_in: list[Port] = []
        built_out: list[Port] = []

        # Input ports — left edge
        for i, (attr_name, label, mtype) in enumerate(in_ports):
            py = y + self.TITLE_H + self.PORT_SPACING * (i + 0.5) + self.PAD // 2
            px = x
            ov = c.create_oval(
                px - self.PORT_R, py - self.PORT_R, px + self.PORT_R, py + self.PORT_R,
                fill="#aaaaaa", outline="#555555", tags=(tag, f"inport:{name}:{attr_name}"),
            )
            item_ids.append(ov)
            txt = c.create_text(px + self.PORT_R + 4, py, text=label, anchor="w",
                                 font=("TkDefaultFont", 8), tags=tag)
            item_ids.append(txt)
            built_in.append(Port(attr_name, label, ov, float(px), float(py), mtype))

        # Output ports — right edge
        for i, (attr_name, label, mtype) in enumerate(out_ports):
            py = y + self.TITLE_H + self.PORT_SPACING * (i + 0.5) + self.PAD // 2
            px = x + w
            ov = c.create_oval(
                px - self.PORT_R, py - self.PORT_R, px + self.PORT_R, py + self.PORT_R,
                fill="#44aa44", outline="#226622", tags=(tag, f"outport:{name}:{attr_name}"),
            )
            item_ids.append(ov)
            txt = c.create_text(px - self.PORT_R - 4, py, text=label, anchor="e",
                                 font=("TkDefaultFont", 8), tags=tag)
            item_ids.append(txt)
            built_out.append(Port(attr_name, label, ov, float(px), float(py), mtype))

            # Left-click: start connection drag. Shift+Left-click: toggle recording.
            c.tag_bind(
                ov, "<ButtonPress-1>",
                lambda e, bn=name, pa=attr_name, cx=float(px), cy=float(py):
                    self._on_port_press(e, bn, pa, cx, cy),
            )

        # Block drag bindings on rect and title
        for item in (rect, title):
            c.tag_bind(item, "<ButtonPress-1>", lambda e, n=name: self._on_block_press(e, n))

        block = Block(name, obj, built_out, built_in, item_ids, float(x), float(y), float(w), float(h))
        return block

    # ------------------------------------------------------------------
    # Port lookup
    # ------------------------------------------------------------------

    def _get_port(self, block_name, direction, attr_name):
        block = self._blocks.get(block_name)
        if not block:
            return None
        ports = block.out_ports if direction == "out" else block.in_ports
        for p in ports:
            if p.attr_name == attr_name:
                return p
        return None

    # ------------------------------------------------------------------
    # Block drag (canvas-level motion handler)
    # ------------------------------------------------------------------

    def _on_block_press(self, event, name):
        if self._conn_src is not None:
            return
        self._drag_block = name
        self._drag_prev = (event.x, event.y)

    # ------------------------------------------------------------------
    # Connection drag (port press)
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Type-compatibility highlighting
    # ------------------------------------------------------------------

    def _src_msg_type(self) -> str:
        """Return the msg_type of the port currently being dragged, or ''."""
        if not self._conn_src:
            return ""
        bn, pa, _, _ = self._conn_src
        p = self._get_port(bn, "out", pa)
        return p.msg_type if p else ""

    def _highlight_compatible_ports(self, src_type: str):
        """Tint all input port ovals: green if compatible, dim if not."""
        for block in self._blocks.values():
            for p in block.in_ports:
                if not src_type or not p.msg_type or src_type == p.msg_type:
                    self._canvas.itemconfig(p.oval_id, fill="#22cc22", outline="#117711")
                else:
                    self._canvas.itemconfig(p.oval_id, fill="#dddddd", outline="#aaaaaa")

    def _restore_port_colors(self):
        """Restore all input port ovals to their neutral colour."""
        for block in self._blocks.values():
            for p in block.in_ports:
                self._canvas.itemconfig(p.oval_id, fill="#aaaaaa", outline="#555555")

    def _on_port_press(self, event, block_name, attr_name, cx, cy):
        # Shift+click toggles recording on this output port; plain click drags a connection
        if event.state & 0x1:
            self._toggle_record(block_name, attr_name)
            return
        self._drag_block = None
        self._drag_prev = None
        self._conn_src = (block_name, attr_name, cx, cy)
        self._rubber_line = self._canvas.create_line(
            cx, cy, event.x, event.y,
            fill="#ff8800", width=2, dash=(6, 4),
        )
        # Highlight compatible input ports immediately
        self._highlight_compatible_ports(self._src_msg_type())

    # ------------------------------------------------------------------
    # Unified canvas motion / release
    # ------------------------------------------------------------------

    def _on_canvas_drag(self, event):
        if self._conn_src:
            if self._rubber_line:
                src_cx, src_cy = self._conn_src[2], self._conn_src[3]
                self._canvas.coords(self._rubber_line, src_cx, src_cy, event.x, event.y)

        elif self._drag_block and self._drag_prev:
            dx = event.x - self._drag_prev[0]
            dy = event.y - self._drag_prev[1]
            self._drag_prev = (event.x, event.y)

            block = self._blocks.get(self._drag_block)
            if block:
                self._canvas.move(f"block:{self._drag_block}", dx, dy)
                block.x += dx
                block.y += dy
                for p in block.out_ports + block.in_ports:
                    p.cx += dx
                    p.cy += dy
                self._update_connections()

    def _on_canvas_release(self, event):
        if self._conn_src:
            if self._rubber_line:
                self._canvas.delete(self._rubber_line)
                self._rubber_line = None

            src_block_name, src_port_attr, src_cx, src_cy = self._conn_src
            target = self._find_nearest_inport(event.x, event.y)
            if target:
                dst_block_name, dst_port_attr = target
                sp = self._get_port(src_block_name, "out", src_port_attr)
                dp = self._get_port(dst_block_name, "in",  dst_port_attr)

                # Type compatibility check
                if sp and dp and sp.msg_type and dp.msg_type and sp.msg_type != dp.msg_type:
                    self._sim_gui._set_status(
                        "subscribeTo", False,
                        f" type mismatch: {sp.msg_type} ≠ {dp.msg_type}",
                    )
                    # Briefly flash the destination port red to give visual feedback
                    self._canvas.itemconfig(dp.oval_id, fill="#cc2222", outline="#880000")
                    self._canvas.after(
                        400,
                        lambda oid=dp.oval_id: self._canvas.itemconfig(
                            oid, fill="#aaaaaa", outline="#555555"
                        ),
                    )
                else:
                    # Prevent duplicate connections
                    already = any(
                        c.src_block == src_block_name and c.src_port == src_port_attr
                        and c.dst_block == dst_block_name and c.dst_port == dst_port_attr
                        for c in self._connections
                    )
                    if not already and dp:
                        lid = self._draw_conn_line(src_cx, src_cy, dp.cx, dp.cy, applied=False)
                        self._connections.append(
                            Connection(src_block_name, src_port_attr,
                                       dst_block_name, dst_port_attr, lid, False)
                        )
                        self._canvas.tag_lower("connection")

            self._restore_port_colors()
            self._conn_src = None

        elif self._drag_block:
            self._drag_block = None
            self._drag_prev = None

    def _find_nearest_inport(self, x, y):
        """Return (block_name, attr_name) of the closest InPort within SNAP_R, or None."""
        best = None
        best_dist = self.SNAP_R
        for block in self._blocks.values():
            for p in block.in_ports:
                dist = ((p.cx - x) ** 2 + (p.cy - y) ** 2) ** 0.5
                if dist < best_dist:
                    best_dist = dist
                    best = (block.name, p.attr_name)
        return best

    def _on_right_click(self, event):
        items = self._canvas.find_overlapping(event.x - 5, event.y - 5, event.x + 5, event.y + 5)
        for item_id in reversed(items):
            if "connection" in self._canvas.gettags(item_id):
                self._canvas.delete(item_id)
                self._connections = [c for c in self._connections if c.line_id != item_id]
                return

    # ------------------------------------------------------------------
    # Connection line drawing & updating
    # ------------------------------------------------------------------

    def _draw_conn_line(self, x1, y1, x2, y2, applied=False):
        mid_x = (x1 + x2) / 2
        color = "#2266cc" if applied else "#ff8800"
        dash = () if applied else (6, 4)
        return self._canvas.create_line(
            x1, y1, mid_x, y1, mid_x, y2, x2, y2,
            smooth=True, fill=color, width=2, dash=dash, tags="connection",
        )

    def _update_connections(self):
        for conn in self._connections:
            sp = self._get_port(conn.src_block, "out", conn.src_port)
            dp = self._get_port(conn.dst_block, "in", conn.dst_port)
            if sp and dp:
                mid_x = (sp.cx + dp.cx) / 2
                self._canvas.coords(conn.line_id, sp.cx, sp.cy, mid_x, sp.cy, mid_x, dp.cy, dp.cx, dp.cy)
        self._canvas.tag_lower("connection")

    # ------------------------------------------------------------------
    # Apply connections
    # ------------------------------------------------------------------

    def _apply_connections(self):
        errors = []
        for conn in self._connections:
            if conn.applied:
                continue
            try:
                src_obj = self._blocks[conn.src_block].obj
                src_msg = src_obj if conn.src_port == "__self__" else getattr(src_obj, conn.src_port)

                dst_obj = self._blocks[conn.dst_block].obj
                dst_msg = getattr(dst_obj, conn.dst_port)

                dst_msg.subscribeTo(src_msg)
                conn.applied = True

                # Redraw as blue solid line
                sp = self._get_port(conn.src_block, "out", conn.src_port)
                dp = self._get_port(conn.dst_block, "in", conn.dst_port)
                if sp and dp:
                    self._canvas.delete(conn.line_id)
                    conn.line_id = self._draw_conn_line(sp.cx, sp.cy, dp.cx, dp.cy, applied=True)

            except Exception as ex:
                errors.append(f"{conn.src_block}.{conn.src_port} \u2192 {conn.dst_block}.{conn.dst_port}: {ex}")

        self._canvas.tag_lower("connection")

        if errors:
            self._sim_gui._set_status("subscribeTo", False, errors[0])
        else:
            self._sim_gui._set_status("subscribeTo", True)

    # ------------------------------------------------------------------
    # Recording — toggle via right-click context menu
    # ------------------------------------------------------------------

    def _toggle_record(self, block_name, attr_name):
        """Toggle recording on an output port (called on Shift+Left-click)."""
        if (block_name, attr_name) in self._recorders:
            self._disarm_recorder(block_name, attr_name)
        else:
            self._arm_recorder(block_name, attr_name)

    def _arm_recorder(self, block_name, attr_name):
        key = (block_name, attr_name)
        if key in self._recorders:
            return
        port = self._get_port(block_name, "out", attr_name)
        if not port:
            return
        label = f"{block_name} \u00b7 {port.label}"
        ring_id = self._draw_recorder_ring(port.cx, port.cy, block_name, attr_name, recorded=False)
        entry = RecEntry(
            block_name=block_name,
            port_attr=attr_name,
            label=label,
            ring_id=ring_id,
            rec_obj=None,
        )
        self._recorders[key] = entry
        self._add_rec_panel_row(key, entry)

    def _disarm_recorder(self, block_name, attr_name):
        key = (block_name, attr_name)
        entry = self._recorders.pop(key, None)
        if not entry:
            return
        if entry.ring_id != -1:
            self._canvas.delete(entry.ring_id)
        self._remove_rec_panel_row(key)

    def _draw_recorder_ring(self, cx, cy, block_name, attr_name, recorded=False):
        r = self.PORT_R + 5
        fill = "#1a7a1a" if recorded else "#e65c00"
        outline = "#0f5a0f" if recorded else "#cc4400"
        ring_id = self._canvas.create_oval(
            cx - r, cy - r, cx + r, cy + r,
            fill=fill, outline=outline, width=2,
            tags=f"recorder_ring:{block_name}:{attr_name}",
        )
        self._canvas.tag_lower(ring_id)
        return ring_id

    # ------------------------------------------------------------------
    # Recording panel rows
    # ------------------------------------------------------------------

    def _add_rec_panel_row(self, key, entry):
        row = ttk.Frame(self._rec_list_frame)
        row.pack(fill=tk.X, pady=2)

        dot = tk.Label(row, text="\u25cf", foreground="#e65c00",
                       font=("TkDefaultFont", 11), width=2)
        dot.pack(side=tk.LEFT)

        ttk.Label(row, text=entry.label, anchor="w").pack(side=tk.LEFT, padx=(2, 4))

        remove_btn = ttk.Button(
            row, text="\u2715", width=2,
            command=lambda k=key: self._disarm_recorder(k[0], k[1]),
        )
        remove_btn.pack(side=tk.RIGHT)

        self._rec_panel_rows[key] = (row, dot)

    def _remove_rec_panel_row(self, key):
        widgets = self._rec_panel_rows.pop(key, None)
        if widgets:
            widgets[0].destroy()

    def _update_rec_panel_dot(self, key, recorded: bool):
        widgets = self._rec_panel_rows.get(key)
        if widgets:
            widgets[1].config(foreground="#1a7a1a" if recorded else "#e65c00")

    # ------------------------------------------------------------------
    # Apply recorders (called before run_sim and via button)
    # ------------------------------------------------------------------

    def _apply_recorders(self):
        sim = self._sim_gui.sim
        if sim is None:
            return
        errors = []
        for key, entry in self._recorders.items():
            if entry.rec_obj is not None:
                continue  # already attached from a previous call
            block = self._blocks.get(entry.block_name)
            if not block:
                errors.append(
                    f"'{entry.block_name}' not on canvas — refresh the Messages tab first."
                )
                continue
            try:
                msg = (block.obj
                       if entry.port_attr == "__self__"
                       else getattr(block.obj, entry.port_attr))
                rec = msg.recorder()
                sim.scSim.AddModelToTask("DynamicsTask", rec)
                sim.recording.append(rec)
                entry.rec_obj = rec
            except Exception as ex:
                errors.append(f"{entry.label}: {ex}")

        count = sum(1 for e in self._recorders.values() if e.rec_obj is not None)
        if errors:
            self._sim_gui._set_status("recorders", False, f" — {errors[0]}")
        elif count:
            self._sim_gui._set_status("recorders", True,
                                      f" — {count} recorder(s) attached")

    # ------------------------------------------------------------------
    # Post-run export
    # ------------------------------------------------------------------

    def _export_recordings(self):
        active = {k: e for k, e in self._recorders.items() if e.rec_obj is not None}
        if not active:
            return

        save_dir = (self._save_dir_var.get().strip()
                    if self._save_dir_var else "./rec_output")
        if not save_dir:
            save_dir = "./rec_output"
        os.makedirs(save_dir, exist_ok=True)

        saved, errors = 0, []
        for key, entry in active.items():
            try:
                rec = entry.rec_obj
                times = np.array(rec.times())
                data: dict = {"times_ns": times}

                for attr in dir(rec):
                    if attr.startswith("_") or attr == "times":
                        continue
                    try:
                        val = getattr(rec, attr)
                        if callable(val):
                            continue
                        arr = np.array(val)
                        if arr.size > 0:
                            data[attr] = arr
                    except Exception:
                        pass

                safe = entry.label.replace(" ", "_").replace("\u00b7", "").replace("/", "_")
                fpath = os.path.join(save_dir, f"{safe}.npz")
                np.savez(fpath, **data)
                saved += 1

                # Visual: ring and panel dot turn green
                if entry.ring_id != -1:
                    self._canvas.itemconfig(
                        entry.ring_id, fill="#1a7a1a", outline="#0f5a0f",
                    )
                self._update_rec_panel_dot(key, recorded=True)

            except Exception as ex:
                errors.append(f"{entry.label}: {ex}")

        if errors:
            self._sim_gui._set_status("export", False, f" — {errors[0]}")
        elif saved:
            self._sim_gui._set_status(
                "export", True, f" — {saved} file(s) saved to {save_dir}",
            )

    # ------------------------------------------------------------------
    # Save directory browse
    # ------------------------------------------------------------------

    def _browse_save_dir(self):
        d = filedialog.askdirectory(title="Select recording save directory")
        if d and self._save_dir_var:
            self._save_dir_var.set(d)

    # ------------------------------------------------------------------
    # Reset (called from SimGUI._restart)
    # ------------------------------------------------------------------

    def reset_recorders(self):
        """Clear all recorder state: data structures and panel row widgets."""
        self._recorders.clear()
        for widgets in list(self._rec_panel_rows.values()):
            widgets[0].destroy()
        self._rec_panel_rows.clear()


# ---------------------------------------------------------------------------
# Main GUI
# ---------------------------------------------------------------------------

_EXECUTION_METHODS = {"run_sim", "perform_montecarlo"}   # live in their own tabs


class SimGUI:
    def __init__(self):
        self.sim = None
        self.apply_buttons: dict[str, ttk.Button] = {}   # method_name -> Apply button
        self._messages_tab: MessagesTab | None = None
        self._section_dots: dict[str, tk.Label] = {}     # method_name -> status dot label
        # method_name -> (widgets_dict, kwargs_text_widget|None)
        self._method_widgets: dict = {}
        # method_name -> {param_name -> tk.Listbox}  (only for DROPDOWN_LIST params)
        self._listbox_refs: dict = {}
        # Ordered log of every successful _apply call → kwargs (plain Python only).
        # Used by MonteCarlo to rebuild a fresh SimBase for each run.
        self._sim_call_log: dict = {}

        self.root = tk.Tk()
        self.root.title("Basilisk SimBase Controller")
        self.root.geometry("900x800")
        self.root.resizable(True, True)

        self._build_ui()
        self.root.mainloop()

    # ------------------------------------------------------------------
    # Introspection helpers (Configure tab)
    # ------------------------------------------------------------------

    def _get_ordered_methods(self):
        """Return SimBase public methods sorted by source line number, excluding IGNORE methods."""
        methods = []
        for name, func in inspect.getmembers(SimBase, predicate=inspect.isfunction):
            if name.startswith("_") and name != "__init__":
                continue
            doc = inspect.getdoc(func)
            if doc and doc.strip().splitlines()[0].strip().upper() == "IGNORE":
                continue
            try:
                _, lineno = inspect.getsourcelines(func)
            except OSError:
                lineno = 9999
            methods.append((lineno, name, func))
        methods.sort(key=lambda t: t[0])
        return [(name, func) for _, name, func in methods]

    def _get_method_func(self, name: str):
        """Return the unbound function for a SimBase method by name, or None."""
        for mname, func in inspect.getmembers(SimBase, predicate=inspect.isfunction):
            if mname == name:
                return func
        return None

    def _make_scrollable_frame(self, parent) -> tuple:
        """Build a vertically scrollable canvas inside *parent*.

        Returns (canvas, inner_frame) – add widgets to inner_frame.
        """
        canvas = tk.Canvas(parent, borderwidth=0, highlightthickness=0)
        sb = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

        def _mw(e):
            if e.num == 4:
                canvas.yview_scroll(-1, "units")
            elif e.num == 5:
                canvas.yview_scroll(1, "units")
            else:
                canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")

        canvas.bind("<Enter>",  lambda _e: (canvas.bind_all("<MouseWheel>", _mw),
                                            canvas.bind_all("<Button-4>", _mw),
                                            canvas.bind_all("<Button-5>", _mw)))
        canvas.bind("<Leave>",  lambda _e: (canvas.unbind_all("<MouseWheel>"),
                                            canvas.unbind_all("<Button-4>"),
                                            canvas.unbind_all("<Button-5>")))
        return canvas, inner

    def _parse_required(self, func) -> str | None:
        """
        Return 'REQUIRED', 'OPTIONAL', or None based on the first non-blank
        line of the function's docstring.
        """
        doc = inspect.getdoc(func)
        if not doc:
            return None
        first_line = doc.strip().splitlines()[0].strip().upper()
        if first_line == "REQUIRED":
            return "REQUIRED"
        if first_line == "OPTIONAL":
            return "OPTIONAL"
        return None

    def _parse_docstring(self, doc):
        """
        Extract parameter descriptions, DROPDOWN, and DROPDOWN_LIST options.

        Returns:
            descriptions    {param_name: description_string}
            dropdowns       {param_name: [option, ...]}   single-select combobox
            dropdown_lists  {param_name: [option, ...]}   multi-select list builder

        Syntax (indented under the parameter line):
            param_name - description
                DROPDOWN: option1, option2, option3
            param_name - description
                DROPDOWN_LIST: option1, option2, option3
        """
        if not doc:
            return {}, {}, {}

        param_re = re.compile(r"^\s{2,}(\w+)[\s\[\(,].*?[-=]\s+(.+)$")
        dropdown_re = re.compile(r"^\s+DROPDOWN:\s*(.+)$")
        dropdown_list_re = re.compile(r"^\s+DROPDOWN_LIST:\s*(.+)$")
        section_headers = re.compile(r"Parameters|Optional kwargs", re.IGNORECASE)

        descriptions: dict[str, str] = {}
        dropdowns: dict[str, list[str]] = {}
        dropdown_lists: dict[str, list[str]] = {}
        in_params = False
        last_param: str | None = None

        def _scan(lines):
            nonlocal in_params, last_param
            for line in lines:
                stripped = line.strip()
                if section_headers.match(stripped):
                    in_params = True
                    continue
                if not in_params:
                    continue
                dl_m = dropdown_list_re.match(line)
                if dl_m and last_param:
                    opts = [o.strip() for o in dl_m.group(1).split(",") if o.strip()]
                    dropdown_lists[last_param] = opts
                    continue
                d_m = dropdown_re.match(line)
                if d_m and last_param:
                    opts = [o.strip() for o in d_m.group(1).split(",") if o.strip()]
                    dropdowns[last_param] = opts
                    continue
                m = param_re.match(line)
                if m:
                    last_param = m.group(1)
                    descriptions[last_param] = m.group(2).strip()

        _scan(doc.splitlines())

        if not descriptions:
            in_params = True
            last_param = None
            _scan(doc.splitlines())

        return descriptions, dropdowns, dropdown_lists

    def _annotation_str(self, annotation) -> str:
        if annotation is type(None) or annotation is inspect.Parameter.empty:
            return ""
        if hasattr(annotation, "__name__"):
            return annotation.__name__
        return str(annotation)

    def _coerce_value(self, raw: str, annotation, default):
        raw = raw.strip()
        if raw == "" or raw.lower() == "none":
            return None
        if annotation is bool or isinstance(default, bool):
            return raw.lower() in ("1", "true", "yes")
        if annotation is int or isinstance(default, int):
            return int(raw)
        if annotation is float or isinstance(default, float):
            return float(raw)
        if annotation is list or isinstance(default, list):
            return ast.literal_eval(raw)
        if annotation is str or isinstance(default, str):
            return raw
        try:
            return ast.literal_eval(raw)
        except Exception:
            return raw

    # ------------------------------------------------------------------
    # Configure tab — method frame builder
    # ------------------------------------------------------------------

    def _build_method_frame(self, method_name: str, method_func, parent):
        sig = inspect.signature(method_func)
        descriptions, dropdowns, dropdown_lists = self._parse_docstring(inspect.getdoc(method_func))

        title = method_name.replace("_", " ").title()
        frame = ttk.LabelFrame(parent, text=title, padding=(10, 6))
        frame.pack(fill=tk.X, expand=True, padx=10, pady=6)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, minsize=18)   # narrow badge column

        # Legend row
        legend_lbl = ttk.Label(
            frame,
            text=" * required field",
            foreground="#cc0000", font=("TkDefaultFont", 7),
        )
        legend_lbl.grid(row=0, column=0, columnspan=3, sticky="e", pady=(0, 2))

        widgets = {}
        kwargs_text = None
        row = 1   # start after legend row

        for param_name, param in sig.parameters.items():
            if param_name == "self":
                continue

            annotation = param.annotation if param.annotation is not inspect.Parameter.empty else type(None)
            is_required = param.default is inspect.Parameter.empty and \
                          param.kind not in (inspect.Parameter.VAR_KEYWORD,
                                             inspect.Parameter.VAR_POSITIONAL)
            default = param.default if param.default is not inspect.Parameter.empty else None

            if param.kind == inspect.Parameter.VAR_KEYWORD:
                lbl = ttk.Label(frame, text="Extra kwargs\n(key=value, one per line):", anchor="nw")
                lbl.grid(row=row, column=0, sticky="nw", padx=(0, 8), pady=2)
                kwargs_text = tk.Text(frame, height=5, width=35, font=("TkFixedFont", 9))
                kwargs_text.grid(row=row, column=1, sticky="ew", pady=2)
                tip = "Optional kwargs applied to ALL RWs.\nEnter one per line as key=value\n(e.g. Omega=100, U_max=0.2)"
                ToolTip(kwargs_text, tip)
                ToolTip(lbl, tip)
                row += 1
                continue

            type_str = self._annotation_str(annotation)
            lbl_text = f"{param_name} ({type_str}):" if type_str else f"{param_name}:"
            lbl = ttk.Label(frame, text=lbl_text, anchor="w")
            lbl.grid(row=row, column=0, sticky="w", padx=(0, 8), pady=2)

            # Required / optional badge in column 2
            if is_required:
                badge = tk.Label(
                    frame, text="*",
                    foreground="#cc0000", font=("TkDefaultFont", 11, "bold"),
                    anchor="center",
                )
                badge.grid(row=row, column=2, sticky="w", padx=(2, 0))
                ToolTip(badge, f"'{param_name}' is required — it has no default value.")

            dropdown_opts = dropdowns.get(param_name)
            dropdown_list_opts = dropdown_lists.get(param_name)

            if annotation is bool or isinstance(default, bool):
                var = tk.BooleanVar(value=bool(default) if default is not None else False)
                widget = ttk.Checkbutton(frame, variable=var)
                widget.grid(row=row, column=1, sticky="w", pady=2)
                widgets[param_name] = (var, annotation, default)

            elif dropdown_list_opts:
                # ---- DROPDOWN_LIST: listbox above + combobox + add/remove below ----
                tip_base = descriptions.get(param_name, "")
                ToolTip(lbl, tip_base)
                lbl.grid(row=row, column=0, sticky="nw", padx=(0, 8), pady=2)  # re-anchor to nw for tall widget

                # Listbox showing currently selected items
                lb_frame = ttk.Frame(frame)
                lb_frame.grid(row=row, column=1, sticky="ew", pady=(2, 0))
                lb_frame.columnconfigure(0, weight=1)

                listbox = tk.Listbox(
                    lb_frame, height=4, selectmode=tk.SINGLE,
                    font=("TkFixedFont", 9), relief=tk.SOLID, borderwidth=1,
                )
                listbox.grid(row=0, column=0, sticky="ew")
                lb_scroll = ttk.Scrollbar(lb_frame, orient="vertical", command=listbox.yview)
                lb_scroll.grid(row=0, column=1, sticky="ns")
                listbox.config(yscrollcommand=lb_scroll.set)

                # Combobox + buttons row (below listbox, same column)
                ctrl_row = row + 1
                ctrl_frame = ttk.Frame(frame)
                ctrl_frame.grid(row=ctrl_row, column=1, sticky="ew", pady=(2, 4))
                ctrl_frame.columnconfigure(0, weight=1)

                combo_var = tk.StringVar()
                combo = ttk.Combobox(
                    ctrl_frame, textvariable=combo_var,
                    values=dropdown_list_opts, width=28, state="readonly",
                )
                combo.grid(row=0, column=0, sticky="ew", padx=(0, 4))

                def _add_item(lb=listbox, cv=combo_var):
                    val = cv.get()
                    if val and val not in lb.get(0, tk.END):
                        lb.insert(tk.END, val)

                def _remove_item(lb=listbox):
                    sel = lb.curselection()
                    if sel:
                        lb.delete(sel[0])

                ttk.Button(ctrl_frame, text="Add", width=5,
                           command=_add_item).grid(row=0, column=1, padx=(0, 2))
                ttk.Button(ctrl_frame, text="Remove", width=7,
                           command=_remove_item).grid(row=0, column=2)

                ToolTip(combo, f"{tip_base}\n\nSelect items one at a time and click Add.".strip())

                # StringVar whose value is the Python list repr — read by _coerce_value
                list_var = tk.StringVar(value="[]")

                def _sync_list_var(lb=listbox, lv=list_var, *_):
                    lv.set(repr(list(lb.get(0, tk.END))))

                listbox.bind("<<ListboxSelect>>", _sync_list_var)
                # Also sync after add/remove by wrapping the commands
                orig_add, orig_rem = _add_item, _remove_item

                def _add_and_sync(lb=listbox, lv=list_var, cv=combo_var):
                    orig_add(lb, cv)
                    lv.set(repr(list(lb.get(0, tk.END))))

                def _rem_and_sync(lb=listbox, lv=list_var):
                    orig_rem(lb)
                    lv.set(repr(list(lb.get(0, tk.END))))

                # Re-bind buttons with sync versions
                for child in ctrl_frame.winfo_children():
                    if isinstance(child, ttk.Button):
                        if child.cget("text") == "Add":
                            child.config(command=_add_and_sync)
                        elif child.cget("text") == "Remove":
                            child.config(command=_rem_and_sync)

                widgets[param_name] = (list_var, list, default)
                # Store listbox ref so _load_params can repopulate it
                self._listbox_refs.setdefault(method_name, {})[param_name] = listbox
                row += 2   # listbox row + controls row
                continue

            elif dropdown_opts:
                # Render a Combobox; the StringVar holds the selected value(s)
                default_str = "" if default is None else str(default)
                var = tk.StringVar(value=default_str)
                widget = ttk.Combobox(
                    frame, textvariable=var,
                    values=dropdown_opts, width=36, state="readonly",
                )
                widget.grid(row=row, column=1, sticky="ew", pady=2)
                widgets[param_name] = (var, annotation, default)
                tip_base = descriptions.get(param_name, "")
                ToolTip(widget, f"{tip_base}\n\nChoose one option from the dropdown.".strip())
                ToolTip(lbl, tip_base)
                row += 1
                continue
            else:
                default_str = "" if default is None else str(default)
                var = tk.StringVar(value=default_str)
                widget = ttk.Entry(frame, textvariable=var, width=38)
                widget.grid(row=row, column=1, sticky="ew", pady=2)
                widgets[param_name] = (var, annotation, default)

            tip = descriptions.get(param_name, "")
            if tip:
                ToolTip(widget, tip)
                ToolTip(lbl, tip)

            row += 1

        is_init = method_name == "__init__"
        btn_state = tk.NORMAL if is_init else tk.DISABLED
        apply_btn = ttk.Button(
            frame, text="Apply",
            command=lambda mn=method_name, w=widgets, kw=kwargs_text: self._apply(mn, w, kw),
            state=btn_state,
        )
        apply_btn.grid(row=row, column=0, columnspan=3, pady=(8, 2))

        self.apply_buttons[method_name] = apply_btn
        # Keep references so _save_params / _load_params can access all values
        self._method_widgets[method_name] = (widgets, kwargs_text)

    def _apply(self, method_name: str, widgets: dict, kwargs_widget):
        try:
            call_kwargs = {}
            for param_name, (var, annotation, default) in widgets.items():
                raw = var.get()
                call_kwargs[param_name] = self._coerce_value(str(raw), annotation, default)

            extra_kwargs = {}
            if kwargs_widget is not None:
                for line in kwargs_widget.get("1.0", tk.END).strip().splitlines():
                    line = line.strip()
                    if "=" in line:
                        k, _, v = line.partition("=")
                        try:
                            extra_kwargs[k.strip()] = ast.literal_eval(v.strip())
                        except Exception:
                            extra_kwargs[k.strip()] = v.strip()

            if method_name == "__init__":
                self.sim = SimBase(**call_kwargs)
                for name, btn in self.apply_buttons.items():
                    if name != "__init__":
                        btn.config(state=tk.NORMAL)
            elif method_name == "run_sim":
                # Attach any armed recorders before InitializeSimulation is called
                if self._messages_tab:
                    self._messages_tab._apply_recorders()
                method = getattr(self.sim, method_name)
                method(**call_kwargs, **extra_kwargs)
                # Export recorded data after ExecuteSimulation returns
                if self._messages_tab:
                    self._messages_tab._export_recordings()
            else:
                method = getattr(self.sim, method_name)
                method(**call_kwargs, **extra_kwargs)

            self._set_status(method_name, success=True)

            # Log this call so MonteCarlo can rebuild the sim from scratch.
            # Skip execution methods (run_sim, perform_montecarlo) — they are
            # not part of the sim-construction recipe.
            if method_name not in _EXECUTION_METHODS:
                self._sim_call_log[method_name] = {**call_kwargs, **extra_kwargs}

            # Lock the button after a successful apply; execution methods may be re-run freely
            if method_name not in _EXECUTION_METHODS:
                self.apply_buttons[method_name].config(state=tk.DISABLED)

        except Exception as e:
            self._set_status(method_name, success=False, msg=str(e))
            traceback.print_exc()

    def _set_status(self, method_name: str, success: bool, msg: str = ""):
        color = "#1a7a1a" if success else "#cc0000"
        if success:
            text = f"[{method_name}] \u2713{msg}"
        else:
            text = f"[{method_name}] Error: {msg}"
        self.status_var.set(text)
        self.status_label.config(foreground=color)

        dot = self._section_dots.get(method_name)
        if dot:
            if success:
                dot.config(text="\u2713", foreground="#1a7a1a", font=("TkDefaultFont", 11, "bold"))
            else:
                dot.config(text="\u2717", foreground="#cc0000", font=("TkDefaultFont", 11, "bold"))

    def _restart(self):
        """Destroy the C++ sim objects and reset the GUI to its initial state."""
        if self.sim is not None:
            # Explicitly delete to trigger Basilisk's C++ destructors before GC
            del self.sim
            self.sim = None

        # Clear the call log so stale config is not passed to MonteCarlo
        self._sim_call_log.clear()

        # Reset apply buttons: only __init__ re-enabled
        for name, btn in self.apply_buttons.items():
            btn.config(state=tk.NORMAL if name == "__init__" else tk.DISABLED)

        # Reset all status dots to pending
        for dot in self._section_dots.values():
            dot.config(text="\u25cf", foreground="#aaaaaa", font=("TkDefaultFont", 12))

        # Clear recorder state and Messages tab canvas
        if self._messages_tab is not None:
            self._messages_tab.reset_recorders()
            self._messages_tab.refresh()

        self.status_var.set('Ready \u2014 apply "__init__" to begin.')
        self.status_label.config(foreground="black")

    # ------------------------------------------------------------------
    # Parameter persistence (save / load)
    # ------------------------------------------------------------------

    _PARAMS_FILE = "gui_params.json"

    def _save_params(self):
        """Serialize all method widget values to *gui_params.json*."""
        data: dict = {}
        for method_name, (widgets, kwargs_text) in self._method_widgets.items():
            entry: dict = {}
            for param_name, (var, _annotation, _default) in widgets.items():
                # BooleanVar.get() returns bool; everything else is a string
                val = var.get()
                entry[param_name] = val if isinstance(val, bool) else str(val)
            if kwargs_text is not None:
                entry["__kwargs__"] = kwargs_text.get("1.0", tk.END).strip()
            data[method_name] = entry

        with open(self._PARAMS_FILE, "w") as fh:
            json.dump(data, fh, indent=2)

        self.status_var.set(f"Parameters saved to {self._PARAMS_FILE}")
        self.status_label.config(foreground="#1a7a1a")

    def _load_params(self, path: str | None = None, *, silent: bool = False):
        """Populate all widgets from a saved JSON file.

        Parameters
        ----------
        path:
            Path to load from.  Defaults to *gui_params.json* in the cwd.
        silent:
            If True, skip the file-not-found dialog (used for auto-load).
        """
        path = path or self._PARAMS_FILE
        if not os.path.isfile(path):
            if not silent:
                self.status_var.set(f"No saved parameters found at {path}")
                self.status_label.config(foreground="#888888")
            return

        try:
            with open(path) as fh:
                data: dict = json.load(fh)
        except Exception as exc:
            self.status_var.set(f"Load error: {exc}")
            self.status_label.config(foreground="#cc0000")
            return

        for method_name, entry in data.items():
            if method_name not in self._method_widgets:
                continue
            widgets, kwargs_text = self._method_widgets[method_name]
            lb_map = self._listbox_refs.get(method_name, {})

            for param_name, stored_val in entry.items():
                if param_name == "__kwargs__":
                    if kwargs_text is not None:
                        kwargs_text.delete("1.0", tk.END)
                        kwargs_text.insert("1.0", stored_val)
                    continue

                if param_name not in widgets:
                    continue
                var, _annotation, _default = widgets[param_name]

                if isinstance(var, tk.BooleanVar):
                    if isinstance(stored_val, bool):
                        var.set(stored_val)
                    else:
                        var.set(str(stored_val).lower() in ("true", "1"))
                else:
                    var.set(str(stored_val))

                # For DROPDOWN_LIST params the Listbox also needs repopulating
                if param_name in lb_map:
                    lb = lb_map[param_name]
                    lb.delete(0, tk.END)
                    try:
                        items = ast.literal_eval(str(stored_val))
                        if isinstance(items, list):
                            for item in items:
                                lb.insert(tk.END, item)
                    except Exception:
                        pass

        if not silent:
            self.status_var.set(f"Parameters loaded from {path}")
            self.status_label.config(foreground="#1a7a1a")

    def _save_params_as(self):
        path = filedialog.asksaveasfilename(
            title="Save parameters",
            defaultextension=".json",
            filetypes=[("JSON", "*.json"), ("All files", "*")],
            initialfile=self._PARAMS_FILE,
        )
        if path:
            self._PARAMS_FILE = path
            self._save_params()

    def _load_params_from(self):
        path = filedialog.askopenfilename(
            title="Load parameters",
            filetypes=[("JSON", "*.json"), ("All files", "*")],
        )
        if path:
            self._load_params(path)

    # ------------------------------------------------------------------
    # Top-level UI assembly
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Tab builders
    # ------------------------------------------------------------------

    def _build_configure_tab(self, notebook: ttk.Notebook):
        """Build the Configure tab: scrollable method forms + status panel."""
        configure_frame = ttk.Frame(notebook)
        notebook.add(configure_frame, text="Configure")

        container = ttk.Frame(configure_frame)
        container.pack(fill=tk.BOTH, expand=True)

        # Left pane: scrollable method forms (execution methods live elsewhere)
        left_pane = ttk.Frame(container)
        left_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        _, scroll_frame = self._make_scrollable_frame(left_pane)

        for method_name, method_func in self._get_ordered_methods():
            if method_name in _EXECUTION_METHODS:
                continue
            self._build_method_frame(method_name, method_func, scroll_frame)

        # Right pane: configuration status panel (shows ALL methods)
        right_pane = ttk.LabelFrame(container, text="Configuration Status", padding=(12, 8))
        right_pane.pack(side=tk.RIGHT, fill=tk.Y, padx=(4, 8), pady=6)

        ttk.Label(
            right_pane,
            text="● pending   ✓ applied   ✗ error",
            foreground="#888888", font=("TkDefaultFont", 8),
        ).pack(anchor="w", pady=(0, 8))

        ttk.Separator(right_pane, orient="horizontal").pack(fill=tk.X, pady=(0, 6))

        def _add_section_header(text: str):
            ttk.Label(
                right_pane, text=text,
                foreground="#555555", font=("TkDefaultFont", 7, "italic"),
            ).pack(anchor="w", pady=(6, 1))

        _add_section_header("Setup")
        for method_name, method_func in self._get_ordered_methods():
            if method_name in _EXECUTION_METHODS:
                continue
            self._add_status_row(right_pane, method_name, method_func)

        _add_section_header("Execution")
        for method_name in ("run_sim", "perform_montecarlo"):
            func = self._get_method_func(method_name)
            if func is not None:
                self._add_status_row(right_pane, method_name, func)

    def _add_status_row(self, parent, method_name: str, method_func):
        """Add one dot + label + badge row to the status panel."""
        row_frame = ttk.Frame(parent)
        row_frame.pack(fill=tk.X, pady=3)

        dot = tk.Label(
            row_frame, text="●",
            foreground="#aaaaaa", font=("TkDefaultFont", 12),
            width=2, anchor="center",
        )
        dot.pack(side=tk.LEFT)

        title = method_name.replace("_", " ").title()
        ttk.Label(row_frame, text=title, width=18, anchor="w").pack(side=tk.LEFT, padx=(4, 0))

        req = self._parse_required(method_func)
        if req == "REQUIRED":
            badge = tk.Label(
                row_frame, text="REQ",
                background="#b00020", foreground="white",
                font=("TkDefaultFont", 7, "bold"),
                padx=3, pady=1, relief=tk.FLAT,
            )
            badge.pack(side=tk.LEFT, padx=(4, 0))
            ToolTip(badge, "This step is required before running the simulation.")
        elif req == "OPTIONAL":
            badge = tk.Label(
                row_frame, text="OPT",
                background="#1565c0", foreground="white",
                font=("TkDefaultFont", 7, "bold"),
                padx=3, pady=1, relief=tk.FLAT,
            )
            badge.pack(side=tk.LEFT, padx=(4, 0))
            ToolTip(badge, "This step is optional.")

        self._section_dots[method_name] = dot

    def _build_run_sim_tab(self, notebook: ttk.Notebook):
        """Build the Run Sim tab with the run_sim method form."""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Run Sim")

        # Info banner
        banner = ttk.Frame(frame)
        banner.pack(fill=tk.X, padx=8, pady=(8, 0))
        ttk.Label(
            banner,
            text="Configure recorders in the Messages tab before running.",
            foreground="#1565c0", font=("TkDefaultFont", 9, "italic"),
        ).pack(side=tk.LEFT)

        ttk.Separator(frame, orient="horizontal").pack(fill=tk.X, padx=8, pady=6)

        # Scrollable content area
        _, scroll_frame = self._make_scrollable_frame(frame)

        func = self._get_method_func("run_sim")
        if func is not None:
            self._build_method_frame("run_sim", func, scroll_frame)

    def _build_run_mc_tab(self, notebook: ttk.Notebook):
        """Build the Run MC tab: run parameters, dispersion picker, execution panel."""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Run MC")

        # Internal state
        self._mc_disp_vars: dict = {}   # attr_name -> {"enabled": BoolVar, "bounds": [StringVar]}
        self._mc_param_vars: dict = {}  # "num_sims" / "num_threads" / "stop_time"

        _, scroll_frame = self._make_scrollable_frame(frame)

        # ── Run Parameters ────────────────────────────────────────────────────
        params_lf = ttk.LabelFrame(scroll_frame, text="Run Parameters", padding=(12, 8))
        params_lf.pack(fill=tk.X, padx=8, pady=(8, 4))
        params_lf.columnconfigure(1, weight=1)

        for row_i, (key, label, default, hint) in enumerate([
            ("num_sims",    "Num Simulations",  "32",  ""),
            ("num_threads", "Num Threads",       "8",   ""),
            ("stop_time",   "Stop Time [sec]",   "",    "leave blank = full orbit"),
        ]):
            ttk.Label(params_lf, text=label + ":").grid(
                row=row_i, column=0, sticky="w", pady=3, padx=(0, 8))
            var = tk.StringVar(value=default)
            ttk.Entry(params_lf, textvariable=var, width=14).grid(
                row=row_i, column=1, sticky="w")
            if hint:
                ttk.Label(params_lf, text=hint,
                          foreground="#888888", font=("TkDefaultFont", 8)).grid(
                    row=row_i, column=2, sticky="w", padx=(8, 0))
            self._mc_param_vars[key] = var

        ttk.Label(params_lf, text="Output Dir:").grid(
            row=3, column=0, sticky="w", pady=3, padx=(0, 8))
        self._mc_output_var = tk.StringVar(value="mc_output")
        ttk.Entry(params_lf, textvariable=self._mc_output_var).grid(
            row=3, column=1, sticky="ew")
        ttk.Button(params_lf, text="Browse",
                   command=self._mc_browse_output_dir).grid(
            row=3, column=2, padx=(8, 0))

        # ── Dispersions ───────────────────────────────────────────────────────
        disp_lf = ttk.LabelFrame(scroll_frame, text="Dispersions", padding=(12, 8))
        disp_lf.pack(fill=tk.X, padx=8, pady=4)

        info_row = ttk.Frame(disp_lf)
        info_row.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(
            info_row,
            text="Check attributes to disperse. Click Refresh after the spacecraft is built "
                 "to validate availability and show current values.",
            foreground="#555555", font=("TkDefaultFont", 8, "italic"),
            wraplength=580,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(info_row, text="\u27f3  Refresh",
                   command=self._mc_refresh_dispersions).pack(side=tk.RIGHT)

        ttk.Separator(disp_lf, orient="horizontal").pack(fill=tk.X, pady=(4, 6))

        # Column headers
        hdr = ttk.Frame(disp_lf)
        hdr.pack(fill=tk.X, padx=2)
        for col, (text, width) in enumerate([
            ("", 2), ("Attribute", 18), ("Dispersion Type", 26),
            ("Parameters", 28), ("Current Value", 0),
        ]):
            ttk.Label(hdr, text=text, font=("TkDefaultFont", 8, "bold"),
                      foreground="#555555", width=width).grid(
                row=0, column=col, sticky="w", padx=(0, 4))

        ttk.Separator(disp_lf, orient="horizontal").pack(fill=tk.X, pady=(2, 4))

        self._mc_disp_rows_frame = ttk.Frame(disp_lf)
        self._mc_disp_rows_frame.pack(fill=tk.X)
        self._mc_build_disp_rows()

        # ── Execution ─────────────────────────────────────────────────────────
        exec_lf = ttk.LabelFrame(scroll_frame, text="Execution", padding=(12, 8))
        exec_lf.pack(fill=tk.X, padx=8, pady=(4, 12))
        exec_lf.columnconfigure(0, weight=1)

        self._mc_progress = ttk.Progressbar(
            exec_lf, orient="horizontal", mode="determinate")
        self._mc_progress.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))

        self._mc_progress_label = ttk.Label(
            exec_lf, text="Not started",
            foreground="#888888", font=("TkDefaultFont", 8))
        self._mc_progress_label.grid(row=1, column=0, sticky="w", pady=(0, 8))

        btn_row = ttk.Frame(exec_lf)
        btn_row.grid(row=2, column=0, columnspan=2, sticky="w")

        run_btn = tk.Button(
            btn_row,
            text="\u25b6  Run Monte Carlo",
            command=self._mc_run,
            background="#1565c0", foreground="white",
            activebackground="#1976d2", activeforeground="white",
            font=("TkDefaultFont", 10, "bold"),
            relief=tk.FLAT, padx=12, pady=6, cursor="hand2",
        )
        run_btn.pack(side=tk.LEFT)

        ttk.Button(
            btn_row, text="Open Results Folder",
            command=self._open_mc_results,
        ).pack(side=tk.LEFT, padx=(10, 0))

    # ------------------------------------------------------------------
    # MC helper methods
    # ------------------------------------------------------------------

    def _mc_build_disp_rows(self, current_values: dict | None = None):
        """Build (or rebuild) one checkbox row per hub attribute in the registry.

        *current_values* maps attr_name -> string representation of the live value.
        If None the "Current Value" column shows "--".
        """
        for w in self._mc_disp_rows_frame.winfo_children():
            w.destroy()
        self._mc_disp_vars.clear()

        for row_i, (attr_name, bsk_path, disp_cls_fn, defaults, param_specs, desc) in \
                enumerate(_HUB_ATTR_REGISTRY):

            enabled_var = tk.BooleanVar(value=True)
            bound_vars = [tk.StringVar(value=d) for d in defaults]
            self._mc_disp_vars[attr_name] = {
                "enabled": enabled_var,
                "bounds": bound_vars,
                "path": bsk_path,
                "disp_cls_fn": disp_cls_fn,
            }

            row = ttk.Frame(self._mc_disp_rows_frame)
            row.pack(fill=tk.X, pady=2)

            # Checkbox
            ttk.Checkbutton(row, variable=enabled_var).grid(
                row=0, column=0, padx=(0, 4))

            # Attribute name + tooltip for the Basilisk path
            attr_lbl = ttk.Label(
                row, text=attr_name, width=18,
                font=("TkDefaultFont", 9, "bold"))
            attr_lbl.grid(row=0, column=1, sticky="w")
            ToolTip(attr_lbl, f"Basilisk path: {bsk_path}\n{desc}")

            # Auto-selected dispersion class name
            disp_cls = disp_cls_fn()
            type_name = disp_cls.__name__ if disp_cls else "N/A (import error)"
            type_lbl = ttk.Label(
                row, text=type_name, width=26,
                foreground="#1565c0", font=("TkDefaultFont", 9))
            type_lbl.grid(row=0, column=2, sticky="w")
            ToolTip(type_lbl, f"Auto-selected dispersion type for {attr_name}")

            # Parameter entries (bounds for the dispersion constructor)
            param_frame = ttk.Frame(row)
            param_frame.grid(row=0, column=3, sticky="w")
            if not param_specs:
                ttk.Label(param_frame, text="(no params)",
                          foreground="#aaaaaa",
                          font=("TkDefaultFont", 8)).pack(side=tk.LEFT)
            else:
                for i, (lbl, unit) in enumerate(param_specs):
                    ttk.Label(param_frame,
                              text=f"{lbl} [{unit}]:",
                              font=("TkDefaultFont", 8)).pack(side=tk.LEFT, padx=(0, 2))
                    ttk.Entry(param_frame, textvariable=bound_vars[i],
                              width=7).pack(side=tk.LEFT, padx=(0, 8))

            # Current value column (live from scObject.hub after Refresh)
            cur_text = "--"
            if current_values is not None:
                cur_text = current_values.get(attr_name, "N/A")
            cur_lbl = ttk.Label(
                row, text=cur_text,
                foreground="#444444", font=("TkDefaultFont", 8),
                wraplength=200)
            cur_lbl.grid(row=0, column=4, sticky="w", padx=(8, 0))

    def _mc_refresh_dispersions(self):
        """Inspect scObject.hub live and rebuild dispersion rows with current values."""
        current_values: dict | None = None
        if self.sim is not None and hasattr(self.sim, "scObject"):
            hub = self.sim.scObject.hub
            current_values = {}
            for attr_name, *_ in _HUB_ATTR_REGISTRY:
                try:
                    val = getattr(hub, attr_name)
                    # Truncate long representations for readability
                    s = repr(val)
                    current_values[attr_name] = s[:60] + "…" if len(s) > 60 else s
                except AttributeError:
                    current_values[attr_name] = "N/A"
        self._mc_build_disp_rows(current_values)

    def _mc_browse_output_dir(self):
        path = filedialog.askdirectory(title="Select Monte Carlo Output Directory")
        if path:
            self._mc_output_var.set(path)

    def _mc_run(self):
        """Create a MonteCarlo instance from the configured scSim and execute it."""
        if MonteCarlo is None:
            self._mc_progress_label.config(
                text=f"Error: Basilisk MC unavailable — {_MC_IMPORT_ERROR}",
                foreground="#cc0000")
            return
        if self.sim is None:
            self._mc_progress_label.config(
                text="Error: apply __init__ first to build the simulation.",
                foreground="#cc0000")
            return

        try:
            num_sims    = int(self._mc_param_vars["num_sims"].get()    or 32)
            num_threads = int(self._mc_param_vars["num_threads"].get() or 8)
            raw_stop    = self._mc_param_vars["stop_time"].get().strip()
            stop_time   = float(raw_stop) if raw_stop else (
                getattr(self.sim, "T", 5400.0))
            output_dir  = self._mc_output_var.get().strip() or "mc_output"

            # Collect enabled dispersions
            disp_paths: list[str]  = []
            disp_classes: list     = []
            disp_bounds: list[list] = []

            for attr_name, cfg in self._mc_disp_vars.items():
                if not cfg["enabled"].get():
                    continue
                disp_cls = cfg["disp_cls_fn"]()
                if disp_cls is None:
                    continue
                disp_paths.append(cfg["path"])
                disp_classes.append(disp_cls)
                disp_bounds.append([float(v.get()) for v in cfg["bounds"]])

            self._mc_progress_label.config(
                text=f"Running {num_sims} sims on {num_threads} threads…",
                foreground="#1565c0")
            self._mc_progress["value"] = 0
            self.root.update_idletasks()

            mc = MonteCarlo(
                self.sim.scSim, num_sims, num_threads, stop_time, output_dir)

            if disp_paths:
                mc.add_dispersions(disp_paths, disp_classes, disp_bounds)

            mc.build_and_execute()

            self._mc_progress["value"] = 100
            self._mc_output_var.set(output_dir)
            self._mc_progress_label.config(
                text=f"Completed {num_sims} runs \u2192 {output_dir}",
                foreground="#1a7a1a")
            self._set_status("perform_montecarlo", success=True)

        except Exception as exc:
            self._mc_progress_label.config(
                text=f"Error: {exc}", foreground="#cc0000")
            self._set_status("perform_montecarlo", success=False, msg=str(exc))
            traceback.print_exc()

    def _open_mc_results(self):
        """Open the MC output directory in the system file manager."""
        import subprocess
        path = self._mc_output_var.get() or "mc_output"
        if not os.path.isdir(path):
            self._mc_progress_label.config(
                text=f"Directory not found: {path}", foreground="#cc0000")
            return
        try:
            subprocess.Popen(["xdg-open", path])
        except Exception as exc:
            self._mc_progress_label.config(text=f"Error: {exc}", foreground="#cc0000")

    # ------------------------------------------------------------------
    # Top-level UI assembly
    # ------------------------------------------------------------------

    def _build_ui(self):
        if IMPORT_ERROR:
            ttk.Label(
                self.root,
                text=f"Failed to import SimBase:\n{IMPORT_ERROR}",
                foreground="red", wraplength=860,
            ).pack(padx=10, pady=20)
            return

        header = ttk.Frame(self.root)
        header.pack(fill=tk.X, padx=10, pady=(12, 0))

        ttk.Label(
            header, text="Basilisk SimBase Controller",
            font=("TkDefaultFont", 14, "bold"),
        ).pack(side=tk.LEFT)

        restart_btn = tk.Button(
            header, text="\u21ba  Restart Setup",
            command=self._restart,
            background="#8b0000", foreground="white",
            activebackground="#b00000", activeforeground="white",
            font=("TkDefaultFont", 10, "bold"),
            relief=tk.FLAT, padx=10, pady=4, cursor="hand2",
        )
        restart_btn.pack(side=tk.RIGHT)
        ToolTip(restart_btn,
                "Destroy all simulation objects (frees C++ memory) and reset\n"
                "the entire setup back to its initial state.")

        # Save / Load buttons
        load_btn = tk.Button(
            header, text="\U0001f4c2  Load",
            command=self._load_params_from,
            background="#1a5276", foreground="white",
            activebackground="#2471a3", activeforeground="white",
            font=("TkDefaultFont", 10, "bold"),
            relief=tk.FLAT, padx=10, pady=4, cursor="hand2",
        )
        load_btn.pack(side=tk.RIGHT, padx=(0, 6))
        ToolTip(load_btn, "Load parameters from a JSON file.")

        save_btn = tk.Button(
            header, text="\U0001f4be  Save",
            command=self._save_params,
            background="#1e8449", foreground="white",
            activebackground="#27ae60", activeforeground="white",
            font=("TkDefaultFont", 10, "bold"),
            relief=tk.FLAT, padx=10, pady=4, cursor="hand2",
        )
        save_btn.pack(side=tk.RIGHT, padx=(0, 6))
        ToolTip(save_btn, f"Save all current parameters to {self._PARAMS_FILE}\n"
                          "(auto-loaded next time the GUI starts).")

        ttk.Label(
            self.root,
            text='Apply "__init__" first, then configure and run the simulation.',
            foreground="gray",
        ).pack(pady=(4, 6))

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=4, pady=(0, 4))

        self._build_configure_tab(notebook)

        # ---- Messages tab ----
        messages_frame = ttk.Frame(notebook)
        notebook.add(messages_frame, text="Messages")
        self._messages_tab = MessagesTab(messages_frame, self)

        self._build_run_sim_tab(notebook)
        self._build_run_mc_tab(notebook)

        # Auto-refresh Messages tab whenever it becomes active (index 1)
        def _on_tab_change(event):
            if notebook.index(notebook.select()) == 1:
                self._messages_tab.refresh()

        notebook.bind("<<NotebookTabChanged>>", _on_tab_change)

        # ---- Status bar ----
        self.status_var = tk.StringVar(value='Ready \u2014 apply "__init__" to begin.')
        self.status_label = ttk.Label(
            self.root, textvariable=self.status_var,
            relief=tk.SUNKEN, anchor="w", padding=(6, 3),
        )
        self.status_label.pack(fill=tk.X, side=tk.BOTTOM)

        # Auto-load last saved parameters (silent if no file exists)
        self._load_params(silent=True)


if __name__ == "__main__":
    SimGUI()
