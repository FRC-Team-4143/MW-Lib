"""Plots for the on-the-fly trajectory generation GitHub issues."""
import heapq
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyBboxPatch, Polygon, Rectangle

SOL = json.load(open(sys.argv[1]))
OUT = Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)

INK = "#1f1f1f"
QUIET = "#6b6b6b"
GREY = "#b9b9b4"
BLUE = "#2f6fdb"
RED = "#d64545"
GREEN = "#2e9e6a"
ORANGE = "#e08a2c"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 11,
    "axes.edgecolor": "#9a9a95",
    "axes.labelcolor": INK,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "xtick.color": QUIET,
    "ytick.color": QUIET,
    "figure.dpi": 150,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.25,
})

FRONT, SIDE = 0.4191, 0.4572
WHEEL_V_MAX = 628.3185307179587 / 6.5 * 0.0508
F_MAX = min(1.0 * 6.5 / 0.0508, 1.5 * 68.0388555 * 9.8 / 4)


def title(ax, main, sub=None):
    ax.set_title(main, loc="left", fontsize=14, fontweight="bold", color=INK, pad=22 if sub else 10)
    if sub:
        ax.text(0, 1.02, sub, transform=ax.transAxes, fontsize=10.5, color=QUIET, va="bottom")


def times(s):
    return np.concatenate([[0.0], np.cumsum(s["dt"][:-1])])


def footprint(ax, x, y, c, s, color, alpha=1.0, lw=1.2, fill=None):
    pts = np.array([[FRONT, SIDE], [FRONT, -SIDE], [-FRONT, -SIDE], [-FRONT, SIDE]])
    R = np.array([[c, -s], [s, c]])
    p = pts @ R.T + [x, y]
    ax.add_patch(Polygon(p, closed=True, fill=fill is not None, facecolor=fill or "none",
                         edgecolor=color, lw=lw, alpha=alpha))
    nose = np.array([FRONT, 0]) @ R.T + [x, y]
    ax.plot([x, nose[0]], [y, nose[1]], color=color, lw=lw, alpha=alpha)


# 06 path around obstacles -------------------------------------------------
def path_obstacles():
    s = SOL["obstacles"]
    x, y = np.array(s["x"]), np.array(s["y"])
    v = np.hypot(s["vx"], s["vy"])
    fig, ax = plt.subplots(figsize=(10, 4.6))
    for cx, cy in [(4.0, 2.2), (7.0, 2.8)]:
        ax.add_patch(Circle((cx, cy), 0.6, facecolor=RED, alpha=0.15, edgecolor=RED, lw=1.5))
        ax.text(cx, cy, "keep-out", ha="center", va="center", color=RED, fontsize=9.5)
    ax.plot([2, 9], [2, 3], ls=(0, (4, 4)), color=QUIET, lw=1.2)
    ax.text(5.5, 2.62, "straight-line guess\n(fails)", color=QUIET, fontsize=9, ha="center", rotation=8)
    gx, gy = [2, 4.0, 7.0, 9], [2, 1.0, 1.7, 3]
    ax.plot(gx, gy, "o--", color=ORANGE, lw=1.2, ms=5, alpha=0.9)
    ax.text(4.0, 0.72, "hint points\n(from A*)", color=ORANGE, fontsize=9, ha="center", va="top")
    for i in range(0, len(x), 6):
        footprint(ax, x[i], y[i], s["c"][i], s["s"][i], GREY, alpha=0.9, lw=1)
    footprint(ax, x[-1], y[-1], s["c"][-1], s["s"][-1], INK, lw=1.4)
    footprint(ax, x[0], y[0], s["c"][0], s["s"][0], INK, lw=1.4)
    sc = ax.scatter(x, y, c=v, cmap="Blues", vmin=0, vmax=v.max(), s=18, zorder=3, edgecolors="none")
    ax.plot(x, y, color=BLUE, lw=1.6, zorder=2)
    cb = fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.01)
    cb.set_label("speed (m/s)", color=QUIET)
    ax.text(x[0], y[0] - 0.62, "start", ha="center", color=INK)
    ax.text(x[-1] + 0.55, y[-1], "goal", va="center", color=INK)
    ax.set_aspect("equal")
    ax.set_xlim(1.2, 10.0)
    ax.set_ylim(0.2, 4.2)
    ax.set_xlabel("field x (m)")
    ax.set_ylabel("field y (m)")
    title(ax, f"Solver output: 7 m around two obstacles in {sum(s['dt']):.2f} s",
          f"Grey outlines = robot bumpers every 6th sample. Solved in {s['ms']:.0f} ms on a laptop (60 samples).")
    fig.savefig(OUT / "06-path-obstacles.png")
    plt.close(fig)


# 07 speed profile ------------------------------------------------------------
def speed_profile():
    s = SOL["medium"]
    t = times(s)
    v = np.hypot(s["vx"], s["vy"])
    a = np.hypot(s["ax"], s["ay"])
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(9, 5.6), sharex=True, gridspec_kw={"hspace": 0.35})
    ax.plot(t, v, color=BLUE, lw=2.2)
    ax.axhline(WHEEL_V_MAX, color=RED, ls=(0, (4, 4)), lw=1.2)
    ax.text(t[-1], WHEEL_V_MAX + 0.08, f"wheel speed limit {WHEEL_V_MAX:.1f} m/s", ha="right", color=RED, fontsize=9.5)
    ax.fill_between(t, v, color=BLUE, alpha=0.08)
    ax.set_ylabel("speed (m/s)")
    ax.set_ylim(0, WHEEL_V_MAX + 0.6)
    title(ax, f"Speed up, cruise, slow down: 4.5 m in {sum(s['dt']):.2f} s",
          "Rule 1 (motion) + Rule 2 (F = m·a) give this shape on their own. Area under the curve = distance.")
    ax2.plot(t, a, color=GREY, lw=2)
    ax2.set_ylabel("acceleration (m/s²)")
    ax2.set_xlabel("time (s)")
    ax2.text(0.02, 0.85, "hard push at the start and end, coasting in the middle", transform=ax2.transAxes, color=QUIET, fontsize=9.5)
    fig.savefig(OUT / "07-speed-profile.png")
    plt.close(fig)


# 08 wheel forces vs limit -----------------------------------------------------
def wheel_forces():
    s = SOL["medium"]
    t = times(s)
    fx, fy = np.array(s["fx"]).T, np.array(s["fy"]).T
    names = ["front-left", "front-right", "back-left", "back-right"]
    fig, ax = plt.subplots(figsize=(9, 4))
    shades = [BLUE, "#5b8de3", "#88aceb", "#b4cbf3"]
    for m in range(4):
        ax.plot(t, np.hypot(fx[m], fy[m]), color=shades[m], lw=1.8, label=names[m])
    ax.axhline(F_MAX, color=RED, ls=(0, (4, 4)), lw=1.2)
    ax.text(t[-1], F_MAX + 3, f"max push per wheel {F_MAX:.0f} N (motor limit; grip allows 250 N)", ha="right", color=RED, fontsize=9.5)
    ax.set_ylim(0, F_MAX + 25)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("wheel push (N)")
    ax.legend(frameon=False, ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.18))
    title(ax, "The fastest path pushes each wheel right up to its limit",
          "Rule 3: every wheel's force must stay under the dashed line at every sample.")
    fig.savefig(OUT / "08-wheel-forces.png")
    plt.close(fig)


# 09 moving start ------------------------------------------------------------
def moving_start():
    s = SOL["moving"]
    x, y = np.array(s["x"]), np.array(s["y"])
    vx, vy = np.array(s["vx"]), np.array(s["vy"])
    fig, ax = plt.subplots(figsize=(8.5, 5.6))
    ax.plot(x, y, color=BLUE, lw=2)
    for i in range(0, len(x), 6):
        ax.annotate("", xy=(x[i] + vx[i] * 0.12, y[i] + vy[i] * 0.12), xytext=(x[i], y[i]),
                    arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=1.2, alpha=0.8))
    footprint(ax, x[0], y[0], s["c"][0], s["s"][0], INK, lw=1.4)
    footprint(ax, x[-1], y[-1], s["c"][-1], s["s"][-1], INK, lw=1.4)
    ax.annotate("", xy=(2.0, 2.0 - 3.0 * 0.3), xytext=(2.0, 2.0),
                arrowprops=dict(arrowstyle="-|>", color=RED, lw=2.5))
    ax.text(2.12, 1.05, "what we asked for:\nalready moving down at 3 m/s", color=RED, fontsize=10, va="center")
    ax.text(3.2, 2.35, f"what TrajoptLib did: ignored that and\nstarted toward the goal at {math.hypot(vx[0], vy[0]):.1f} m/s",
            color=BLUE, fontsize=10)
    ax.text(x[-1] + 0.5, y[-1], "goal", va="center")
    ax.set_aspect("equal")
    ax.set_xlabel("field x (m)")
    ax.set_ylabel("field y (m)")
    ax.set_xlim(0.8, 6.2)
    ax.set_ylim(0.6, 4.7)
    title(ax, "TrajoptLib can't start from the robot's real velocity",
          "It can only cap the start speed and fix a direction line (either sign). Our Java version must pin vx and vy exactly.")
    fig.savefig(OUT / "09-moving-start.png")
    plt.close(fig)


# 10 solve time vs samples -----------------------------------------------------
def solve_vs_samples():
    N = [20, 40, 80]
    data = {
        "short 1.5 m": [15.6, 31.8, 68.7],
        "medium 4 m": [27.8, 50.3, 83.6],
        "long 9 m": [18.6, 41.4, 101.3],
        "moving start": [31.0, 69.7, 121.8],
        "1 obstacle + hint": [98.2, 151.5, 1011.6],
        "2 obstacles + hint": [149.8, 280.3, 561.3],
    }
    fig, ax = plt.subplots(figsize=(9, 4.8))
    for name, ms in data.items():
        obst = "obstacle" in name
        ax.plot(N, ms, "o-", color=BLUE if obst else GREY, lw=2 if obst else 1.6, ms=5)
        ax.text(81.5, ms[-1], name, va="center", fontsize=9.5, color=BLUE if obst else QUIET)
    ax.axhline(100, color=RED, ls=(0, (4, 4)), lw=1.2)
    ax.text(20, 108, "100 ms goal", color=RED, fontsize=9.5)
    ax.set_yscale("log")
    ax.set_yticks([10, 30, 100, 300, 1000])
    ax.set_yticklabels(["10", "30", "100", "300", "1000"])
    ax.set_xticks(N)
    ax.set_xlim(18, 100)
    ax.set_xlabel("number of samples")
    ax.set_ylabel("solve time on a laptop (ms, log scale)")
    title(ax, "More samples = slower solves; obstacles cost the most",
          "TrajoptLib C++ on an i5-1240P with mainbot's specs. The robot controller will be slower.")
    fig.savefig(OUT / "10-solve-vs-samples.png")
    plt.close(fig)


# A* ---------------------------------------------------------------------------
CELL = 0.1
W, H = 8.0, 4.0
OBST = [("circle", 2.6, 1.8, 0.5), ("rect", 4.6, 0.0, 0.5, 2.6), ("circle", 6.5, 1.5, 0.45)]
HALF = 0.47


def inside(ob, px, py, grow):
    if ob[0] == "circle":
        return math.hypot(px - ob[1], py - ob[2]) <= ob[3] + grow
    _, x0, y0, w, h = ob
    dx = max(x0 - px, 0, px - (x0 + w))
    dy = max(y0 - py, 0, py - (y0 + h))
    return math.hypot(dx, dy) <= grow


def grid(grow):
    nx, ny = int(W / CELL), int(H / CELL)
    g = np.zeros((ny, nx), dtype=bool)
    for j in range(ny):
        for i in range(nx):
            px, py = (i + 0.5) * CELL, (j + 0.5) * CELL
            g[j, i] = any(inside(o, px, py, grow) for o in OBST) or px < HALF or px > W - HALF or py < HALF or py > H - HALF
    return g


def astar(g, start, goal):
    ny, nx = g.shape
    h = lambda c: math.hypot(c[0] - goal[0], c[1] - goal[1])
    openq = [(h(start), 0.0, start)]
    came, cost, seen = {}, {start: 0.0}, set()
    while openq:
        _, d, cur = heapq.heappop(openq)
        if cur in seen:
            continue
        seen.add(cur)
        if cur == goal:
            break
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == dj == 0:
                    continue
                n = (cur[0] + di, cur[1] + dj)
                if not (0 <= n[0] < nx and 0 <= n[1] < ny) or g[n[1], n[0]]:
                    continue
                nd = d + math.hypot(di, dj)
                if nd < cost.get(n, 1e9):
                    cost[n], came[n] = nd, cur
                    heapq.heappush(openq, (nd + h(n), nd, n))
    path = [goal]
    while path[-1] != start:
        path.append(came[path[-1]])
    return path[::-1], seen


def clear(g, a, b):
    steps = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 3) + 1
    for k in range(steps + 1):
        t = k / steps
        i, j = round(a[0] + (b[0] - a[0]) * t), round(a[1] + (b[1] - a[1]) * t)
        if g[j, i]:
            return False
    return True


def simplify(g, path):
    out, k = [path[0]], 0
    while k < len(path) - 1:
        far = k + 1
        for m in range(len(path) - 1, k, -1):
            if clear(g, path[k], path[m]):
                far = m
                break
        out.append(path[far])
        k = far
    return out


def draw_obstacles(ax, grow_outline):
    for o in OBST:
        if o[0] == "circle":
            ax.add_patch(Circle((o[1], o[2]), o[3], facecolor=RED, alpha=0.35, edgecolor=RED, lw=1.5))
            if grow_outline:
                ax.add_patch(Circle((o[1], o[2]), o[3] + HALF, fill=False, edgecolor=RED, ls=(0, (3, 3)), lw=1))
        else:
            ax.add_patch(Rectangle((o[1], o[2]), o[3], o[4], facecolor=RED, alpha=0.35, edgecolor=RED, lw=1.5))
            if grow_outline:
                ax.add_patch(FancyBboxPatch((o[1], o[2]), o[3], o[4], boxstyle=f"round,pad={HALF}", fill=False,
                                            edgecolor=RED, ls=(0, (3, 3)), lw=1))


def show_grid(ax, g, color=RED, alpha=0.16):
    img = np.zeros((*g.shape, 4))
    img[g] = matplotlib.colors.to_rgba(color, alpha)
    ax.imshow(img, origin="lower", extent=[0, W, 0, H], interpolation="nearest")
    for k in np.arange(0, W + 1e-9, CELL * 5):
        ax.axvline(k, color="#e6e6e2", lw=0.5, zorder=0)
    for k in np.arange(0, H + 1e-9, CELL * 5):
        ax.axhline(k, color="#e6e6e2", lw=0.5, zorder=0)


def astar_plots():
    g = grid(HALF)
    # 11 occupancy grid
    fig, ax = plt.subplots(figsize=(10, 5.2))
    show_grid(ax, g)
    draw_obstacles(ax, True)
    footprint(ax, 1.3, 0.9, 1, 0, INK, lw=1.4)
    ax.plot(1.3, 0.9, "o", color=INK, ms=4)
    ax.annotate("treat the robot as a dot...", xy=(1.3, 0.9), xytext=(0.7, 3.35), fontsize=10,
                arrowprops=dict(arrowstyle="-", color=QUIET))
    ax.annotate("...by growing every obstacle\nby half the robot's width (dashed)", xy=(3.05, 2.45), xytext=(2.9, 3.35),
                fontsize=10, arrowprops=dict(arrowstyle="-", color=QUIET))
    ax.text(7.95, 0.08, "walls are grown too", ha="right", color=QUIET, fontsize=9.5)
    ax.set_aspect("equal")
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_xlabel("field x (m)")
    ax.set_ylabel("field y (m)")
    title(ax, "Step 1: a grid of blocked squares, built once at startup",
          f"{g.shape[1]}×{g.shape[0]} squares of {CELL * 100:.0f} cm. Shaded = the robot's center can't go there.")
    fig.savefig(OUT / "11-astar-grid.png")
    plt.close(fig)

    # 12 search + simplify
    start, goal = (12, 10), (74, 30)
    path, seen = astar(g, start, goal)
    simple = simplify(g, path)
    fig, ax = plt.subplots(figsize=(10, 5.2))
    show_grid(ax, g)
    ex = np.zeros(g.shape, dtype=bool)
    for c in seen:
        ex[c[1], c[0]] = True
    img = np.zeros((*g.shape, 4))
    img[ex & ~g] = matplotlib.colors.to_rgba(BLUE, 0.12)
    ax.imshow(img, origin="lower", extent=[0, W, 0, H], interpolation="nearest")
    draw_obstacles(ax, False)
    P = np.array([[(c[0] + 0.5) * CELL, (c[1] + 0.5) * CELL] for c in path])
    S = np.array([[(c[0] + 0.5) * CELL, (c[1] + 0.5) * CELL] for c in simple])
    ax.plot(P[:, 0], P[:, 1], color=GREY, lw=2.5, label=f"raw A* path ({len(path)} squares)")
    ax.plot(S[:, 0], S[:, 1], "o-", color=ORANGE, lw=2, ms=7, label=f"simplified: {len(simple) - 2} hint points")
    ax.plot(*S[0], "o", color=INK, ms=9)
    ax.plot(*S[-1], "o", color=INK, ms=9, mfc="white", mew=2)
    ax.text(S[0][0], S[0][1] - 0.28, "start", ha="center")
    ax.text(S[-1][0], S[-1][1] + 0.2, "goal", ha="center")
    ax.legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=3)
    ax.set_aspect("equal")
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_xlabel("field x (m)")
    title(ax, "Steps 2–3: search the grid, then keep only the corners",
          f"Light blue = squares A* looked at ({len(seen)} of {g.size}). The orange corners become the solver's initial guess.")
    fig.savefig(OUT / "12-astar-search.png")
    plt.close(fig)


# 13 handoff timeline -----------------------------------------------------------
def handoff():
    fig, ax = plt.subplots(figsize=(10, 3.6))
    lanes = {"Robot loop": 2, "Background thread": 1}
    ax.broken_barh([(0, 0.25)], (1.75, 0.5), facecolors=GREEN, alpha=0.55)
    ax.broken_barh([(0.25, 1.35)], (1.75, 0.5), facecolors=BLUE, alpha=0.55)
    ax.broken_barh([(0.0, 0.25)], (0.75, 0.5), facecolors=GREY, alpha=0.9)
    ax.text(0.125, 2, "tractor beam", ha="center", va="center", fontsize=9.5)
    ax.text(0.85, 2, "Choreo follower drives the new path", ha="center", va="center", fontsize=10)
    ax.text(0.125, 1, "A* + solve", ha="center", va="center", fontsize=9.5)
    for k in np.arange(0, 1.6, 0.02):
        ax.plot([k, k], [2.28, 2.36], color=QUIET, lw=0.6)
    ax.text(1.58, 2.42, "a tick every 20 ms: none may run late", ha="right", fontsize=9, color=QUIET)
    ax.annotate("", xy=(0.25, 1.72), xytext=(0.25, 1.28), arrowprops=dict(arrowstyle="-|>", color=INK))
    ax.text(0.265, 1.5, "path ready: switch, starting at the sample for 0.25 s,\nnot sample 0 (the robot already moved)", fontsize=9.5, va="center")
    ax.axvline(0, color=INK, lw=1)
    ax.text(0, 2.62, "button pressed", fontsize=9.5)
    ax.set_yticks(list(lanes.values()))
    ax.set_yticklabels(list(lanes.keys()))
    ax.set_ylim(0.5, 2.75)
    ax.set_xlim(-0.03, 1.6)
    ax.set_xlabel("time (s)")
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y", length=0)
    title(ax, "The robot moves the instant the button is pressed",
          "Example timing: a 250 ms solve. If the solve fails, the green bar just keeps going.")
    fig.savefig(OUT / "13-handoff-timeline.png")
    plt.close(fig)


# 14 replay flow ------------------------------------------------------------------
def replay():
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
    def box(ax, x, y, text, color, w=0.36):
        ax.add_patch(FancyBboxPatch((x - w / 2, y - 0.11), w, 0.22, boxstyle="round,pad=0.02",
                                    facecolor=color, alpha=0.18, edgecolor=color, lw=1.5))
        ax.text(x, y, text, ha="center", va="center", fontsize=9.5)
    def arrow(ax, a, b):
        ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="-|>", color=QUIET, lw=1.2))
    for ax, live in zip(axes, [True, False]):
        ax.set_xlim(0, 1)
        ax.set_ylim(-0.04, 1)
        ax.axis("off")
        ax.set_title("On the robot" if live else "In log replay", loc="left", fontsize=12.5, fontweight="bold")
        box(ax, 0.22, 0.8, "drive-to request", GREY)
        if live:
            box(ax, 0.22, 0.45, "TrajGenIO\nruns the solver", BLUE)
        else:
            box(ax, 0.22, 0.45, "TrajGenIO\nreads the log", GREEN)
        box(ax, 0.75, 0.45, "trajectory\n(logged input)", ORANGE)
        box(ax, 0.75, 0.12, "Choreo follower", GREY)
        arrow(ax, (0.22, 0.68), (0.22, 0.57))
        arrow(ax, (0.41, 0.45), (0.56, 0.45))
        arrow(ax, (0.75, 0.33), (0.75, 0.24))
    axes[1].text(0.02, 0.2, "the solver never runs, so\nreplay is exact and instant", fontsize=9.5, color=QUIET)
    fig.suptitle("Log the result as an input so replay doesn't re-solve", x=0.02, y=1.06, ha="left", fontsize=14, fontweight="bold")
    fig.savefig(OUT / "14-replay.png")
    plt.close(fig)


path_obstacles()
speed_profile()
wheel_forces()
moving_start()
solve_vs_samples()
astar_plots()
handoff()
replay()
print("ok")
