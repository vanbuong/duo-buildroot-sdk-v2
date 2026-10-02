#!/usr/bin/env python3
"""Draws docs/dphy_full_lp_network.png - one D-PHY lane receive network (HS + LP)."""
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

fig, ax = plt.subplots(figsize=(15, 9.5)); ax.set_xlim(-3, 24); ax.set_ylim(-6.5, 11.5); ax.axis("off")
C = "#1f3a5f"; K = dict(color=C, lw=1.8, solid_capstyle="round")
def w(*pts): ax.plot([p[0] for p in pts], [p[1] for p in pts], **K)
def dot(x, y): ax.plot(x, y, "o", color=C, ms=6)
def rv(x, y1, y2, lab):  # vertical resistor between y1 (top) and y2 (bottom)
    m = (y1 + y2) / 2; h = 0.45
    w((x, y1), (x, m + h)); w((x, m - h), (x, y2))
    ax.add_patch(Rectangle((x - .22, m - h), .44, 2 * h, fc="white", ec=C, lw=1.8))
    ax.text(x + .45, m, lab, va="center", fontsize=10, color="#b03030")
def ch(x1, x2, y, lab, above=True):  # series cap on horizontal wire
    m = (x1 + x2) / 2
    w((x1, y), (m - .12, y)); w((m + .12, y), (x2, y))
    ax.plot([m - .12] * 2, [y - .45, y + .45], **K); ax.plot([m + .12] * 2, [y - .45, y + .45], **K)
    ax.text(m, y + (.8 if above else -1.0), lab, ha="center", fontsize=10, color="#b03030")
def gnd(x, y):
    w((x, y), (x, y - .4))
    for i, hw in enumerate((.4, .26, .12)): ax.plot([x - hw, x + hw], [y - .4 - .17 * i] * 2, **K)
def hz(x, y1, y2, lab):  # horizontal resistor
    pass

# Lane wires
P, N = 6, 2
ax.text(-2.9, 10.8, "MIPI D-PHY receive network, one lane  (build 3x: CK, D0, D1  ->  12 FPGA pins)", fontsize=15, weight="bold", color=C)
ax.text(-2.9, 10.0, "Full LP support: HS differential (AC-coupled LVDS) + LP single-ended per wire, GW1NR-9 bank @ 1.8 V", fontsize=11, color="#444")
w((-2, P), (15.3, P)); w((-2, N), (15.3, N))
ax.text(-2.9, P + .35, "DSI_x_P\n(Duo S)", fontsize=10, ha="left", va="bottom"); ax.text(-2.9, N - .35, "DSI_x_N\n(Duo S)", fontsize=10, ha="left", va="top")

# 1) split termination 2x50R + 100n to GND at x=1.5
x = 1.5; dot(x, P); dot(x, N)
rv(x, P, 4.45, "50R"); rv(x, N, 3.55, "50R"); w((x, 4.45), (x, 3.55)) if False else None
w((x, 4.45), (x, 3.55)); dot(x, 4)
ch(x, x + 2.2, 4, "100nF", above=True); gnd(x + 2.2, 4)
ax.text(x - .3, 8.1, "HS termination\n100R diff, split,\nAC-grounded", fontsize=10, ha="center", color="#1a7a3a", weight="bold")

# 2) LP taps at x=6.2
x = 6.2; dot(x, P); dot(x, N)
rv(x, P, 8.3, "470R"); rv(x, N, -0.3, "470R")
w((x, 8.3), (x, 8.7), (15.3, 8.7)); w((x, -0.3), (x, -0.7), (15.3, -0.7))
ax.text(x - .2, 9.15, "LP sense: single-ended, series isolation", fontsize=10, color="#1a7a3a", weight="bold")

# 3) HS AC coupling + bias
ch(8.6, 9.7, P, "100nF"); ch(8.6, 9.7, N, "100nF", above=False)
ax.text(9.15, 7.7, "HS AC-couple", fontsize=10, ha="center", color="#1a7a3a", weight="bold")
x = 12.2; dot(x, P); dot(x, N)
rv(x, P, 4.45, "10k"); rv(x, N, 3.55, "10k"); w((x, 4.45), (x, 3.55)); dot(x, 4)
w((x, 4), (13.6, 4)); ax.text(13.7, 4, "VCM 0.9 V", va="center", fontsize=10, color="#b03030")
ax.text(x + 0.6, 7.0, "HS bias to VCM", fontsize=10, ha="center", color="#1a7a3a", weight="bold")

# FPGA block
ax.add_patch(Rectangle((15.3, -1.6), 6.2, 10.8, fc="#eef3fa", ec=C, lw=2))
ax.text(18.4, 9.75, "Tang Nano 9K  (GW1NR-9)", ha="center", fontsize=12, weight="bold", color=C)
for y, t in ((8.7, "LP_x_P   LVCMOS18 in"), (P, "HS_x_P   LVDS in (+)"), (N, "HS_x_N   LVDS in (-)"), (-.7, "LP_x_N   LVCMOS18 in")):
    ax.text(15.6, y + .2, t, fontsize=10.5, va="bottom", color=C); 
ax.text(18.4, 4, "soft D-PHY RX\nIDES8 1:8 + bit-slip\nLP state machine", ha="center", va="center", fontsize=10.5, color="#555")

# VCM generator (shared)
ax.text(-2.9, -2.1, "VCM generator (shared by all lanes)", fontsize=11, weight="bold", color=C)
vx = 0.5
ax.text(vx, -2.75, "1V8", fontsize=10, ha="center", color="#b03030"); w((vx, -3.0), (vx, -3.3))
rv(vx, -3.3, -4.3, "10k"); dot(vx, -4.3); rv(vx, -4.3, -5.3, "10k"); gnd(vx, -5.3)
w((vx, -4.3), (2.6, -4.3)); ax.plot([2.6, 2.6], [-4.3, -4.75], **K)
ax.plot([2.2, 3.0], [-4.75] * 2, **K); ax.plot([2.2, 3.0], [-5.0] * 2, **K); gnd(2.6, -5.0)
ax.text(3.3, -4.3, "VCM 0.9 V", va="center", fontsize=10, color="#b03030")
# notes
notes = ["Notes",
 "- Split termination: 2x50R + 100nF presents 100R diff in HS but is DC-open in LP, so LP-11 (1.2 V) is not loaded down.",
 "- HS path is AC-coupled: FPGA LVDS input common-mode is set by VCM (0.9 V), not the ~0.2 V D-PHY HS common-mode.",
 "- LP path is DC: 470R isolates the stub; verify LP-11 (1.2 V) vs the bank's VIH (~1.17 V at 1.8 V) - marginal, check IO standard.",
 "- Values are a starting point (typical practice, not from a Gowin reference design): confirm with simulation / the Gowin IO guide.",
 "- Pin count: 3 lanes x (HS_P, HS_N, LP_P, LP_N) = 12 FPGA pins; HS pairs must be true LVDS (_T/_C) pairs."]
for i, t in enumerate(notes):
    ax.text(6.3, -2.1 - 0.62 * i, t, fontsize=10 if i else 11, weight="bold" if i == 0 else None, color=C if i == 0 else "#222")
plt.savefig("dphy_full_lp_network.png", dpi=130, bbox_inches="tight", facecolor="white")
