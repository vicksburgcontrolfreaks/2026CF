"""
Visualize all 2026CF autonomous paths overlaid on the 2026 FRC field.
Run with: python visualize_autos.py
Requires: pip install matplotlib
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
from matplotlib.patches import FancyArrowPatch
import numpy as np

# Field dimensions (meters)
FIELD_W = 16.54
FIELD_H = 8.21

# Alliance wall x positions
BLUE_WALL_X = 0.0
RED_WALL_X  = 16.54

# Speaker target positions (approximate center of speaker opening)
RED_TARGET  = (16.54, 5.55)
BLUE_TARGET = (0.0,   5.55)

# ── Path definitions ──────────────────────────────────────────────────────────
# Each path is a list of (x, y) waypoints in order.
# Heading arrows are drawn at each waypoint but not stored here.

paths = {
    "Red Right\nCollect & Shoot": {
        "color": "#cc2200",
        "start": (12.88, 7.62),
        "waypoints": [
            (12.88, 7.62),
            (10.69, 7.62),  # collector deploy
            (8.78,  6.54),  # collect align
            (8.78,  4.54),  # collect end
            (10.69, 7.50),  # midpoint
            (12.88, 7.62),  # transit
            (13.33, 5.62),  # shoot 1
            (15.88, 7.39),  # outpost
            (13.33, 5.62),  # shoot 2
        ],
    },
    "Red Left\nCollect & Shoot": {
        "color": "#ff6600",
        "start": (12.91, 0.46),
        "waypoints": [
            (12.91, 0.46),
            (10.70, 0.46),  # trench far / collector deploy
            (8.77,  2.05),  # collect align
            (8.77,  3.53),  # collect end
            (10.70, 0.46),  # back through trench
            (12.91, 0.46),  # clear trench
            (13.23, 2.37),  # shoot 1
            (12.91, 0.46),  # back to entrance
            (10.70, 0.46),  # through trench 2
            (8.77,  2.05),
            (8.77,  3.53),
            (10.70, 0.46),
            (12.91, 0.46),
            (13.23, 2.37),  # shoot 2
        ],
    },
    "Red Center\nShoot": {
        "color": "#ff99cc",
        "start": (12.09, 4.00),
        "waypoints": [
            (12.09, 4.00),
            (13.91, 4.00),  # shoot
        ],
    },
    "Blue Right\nCollect & Shoot": {
        "color": "#0044cc",
        "start": (3.61, 0.50),
        "waypoints": [
            (3.61, 0.50),
            (5.85, 0.59),   # collector deploy
            (7.76, 1.67),   # collect align
            (7.76, 3.67),   # collect end
            (5.85, 0.71),   # midpoint
            (3.45, 0.59),   # transit
            (3.21, 2.59),   # shoot 1
            (0.66, 0.62),   # outpost
            (3.21, 2.59),   # shoot 2
        ],
    },
    "Blue Left\nCollect & Shoot": {
        "color": "#00aaff",
        "start": (3.60, 7.62),
        "waypoints": [
            (3.60, 7.62),
            (5.84, 7.75),   # trench far / collector deploy
            (7.77, 6.16),   # collect align
            (7.77, 4.68),   # collect end
            (5.84, 7.75),   # back through trench
            (3.60, 7.62),   # clear trench
            (3.31, 5.84),   # shoot 1
            (3.60, 7.62),   # back to entrance
            (5.84, 7.75),   # through trench 2
            (7.77, 6.16),
            (7.77, 4.68),
            (5.84, 7.75),
            (3.60, 7.62),
            (3.31, 5.84),   # shoot 2
        ],
    },
    "Blue Center\nShoot": {
        "color": "#aaddff",
        "start": (4.45, 4.00),
        "waypoints": [
            (4.45, 4.00),
            (2.63, 4.00),   # shoot
        ],
    },
}

# ── Draw ──────────────────────────────────────────────────────────────────────

fig, ax = plt.subplots(figsize=(18, 9))
ax.set_facecolor("#1a1a2e")
fig.patch.set_facecolor("#0f0f1a")

# Field border
field_rect = patches.Rectangle((0, 0), FIELD_W, FIELD_H,
                                linewidth=2, edgecolor="white", facecolor="#2a3a2a")
ax.add_patch(field_rect)

# Alliance walls
ax.axvline(x=0,       color="#4488ff", linewidth=4, alpha=0.8)
ax.axvline(x=FIELD_W, color="#ff4444", linewidth=4, alpha=0.8)

# Field center line
ax.axvline(x=FIELD_W / 2, color="white", linewidth=1, linestyle="--", alpha=0.3)

# Speaker targets
for tx, ty, col, label in [
    (RED_TARGET[0],  RED_TARGET[1],  "#ff4444", "Red\nSpeaker"),
    (BLUE_TARGET[0], BLUE_TARGET[1], "#4488ff", "Blue\nSpeaker"),
]:
    ax.plot(tx, ty, "*", color=col, markersize=18, zorder=10)
    ax.annotate(label, (tx, ty), textcoords="offset points",
                xytext=(15 if tx < 8 else -55, -15),
                color=col, fontsize=7, fontweight="bold")

# Draw each path
for name, path in paths.items():
    color = path["color"]
    wps   = path["waypoints"]
    xs    = [p[0] for p in wps]
    ys    = [p[1] for p in wps]

    # Path line
    ax.plot(xs, ys, "-o", color=color, linewidth=1.8, markersize=5,
            alpha=0.85, zorder=5)

    # Direction arrows along the path
    for i in range(len(wps) - 1):
        x0, y0 = wps[i]
        x1, y1 = wps[i + 1]
        mx = (x0 + x1) / 2
        my = (y0 + y1) / 2
        dx = (x1 - x0)
        dy = (y1 - y0)
        dist = np.hypot(dx, dy)
        if dist > 0.01:
            ax.annotate("", xy=(mx + dx/dist*0.25, my + dy/dist*0.25),
                        xytext=(mx - dx/dist*0.25, my - dy/dist*0.25),
                        arrowprops=dict(arrowstyle="->", color=color,
                                        lw=1.5, alpha=0.7),
                        zorder=6)

    # Start marker
    ax.plot(xs[0], ys[0], "D", color=color, markersize=9, zorder=7,
            markeredgecolor="white", markeredgewidth=0.8)

    # End marker
    ax.plot(xs[-1], ys[-1], "s", color=color, markersize=9, zorder=7,
            markeredgecolor="white", markeredgewidth=0.8)

    # Label near the start
    offset_x = 0.2 if xs[0] < FIELD_W / 2 else -0.2
    offset_y = 0.25
    ax.annotate(name, (xs[0], ys[0]),
                textcoords="offset points",
                xytext=(offset_x * 40, offset_y * 40),
                color=color, fontsize=6.5, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="#0f0f1a", alpha=0.6, ec=color, lw=0.8),
                zorder=8)

# Axis formatting
ax.set_xlim(-0.3, FIELD_W + 0.3)
ax.set_ylim(-0.3, FIELD_H + 0.3)
ax.set_aspect("equal")
ax.set_xlabel("X (meters) — Blue wall at x=0, Red wall at x=16.54",
              color="white", fontsize=9)
ax.set_ylabel("Y (meters)", color="white", fontsize=9)
ax.tick_params(colors="white")
for spine in ax.spines.values():
    spine.set_edgecolor("white")

# Grid
ax.grid(True, color="white", alpha=0.08, linewidth=0.5)
ax.set_xticks(np.arange(0, FIELD_W + 1, 1))
ax.set_yticks(np.arange(0, FIELD_H + 1, 1))

# Legend
legend_elements = [
    mlines.Line2D([0], [0], color=p["color"], linewidth=2, label=n.replace("\n", " "))
    for n, p in paths.items()
]
legend_elements += [
    mlines.Line2D([0], [0], marker="D", color="white", linewidth=0,
                  markersize=7, label="Start pose"),
    mlines.Line2D([0], [0], marker="s", color="white", linewidth=0,
                  markersize=7, label="End pose"),
    mlines.Line2D([0], [0], marker="*", color="white", linewidth=0,
                  markersize=10, label="Speaker target"),
]
ax.legend(handles=legend_elements, loc="upper center",
          bbox_to_anchor=(0.5, -0.08), ncol=4,
          facecolor="#1a1a2e", edgecolor="white",
          labelcolor="white", fontsize=8)

ax.set_title("2026CF Autonomous Paths — All Alliances",
             color="white", fontsize=13, fontweight="bold", pad=12)

plt.tight_layout()
plt.savefig("auto_paths.png", dpi=150, bbox_inches="tight",
            facecolor=fig.get_facecolor())
print("Saved to auto_paths.png")
plt.show()
