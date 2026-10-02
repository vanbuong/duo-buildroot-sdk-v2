#!/usr/bin/env python3
"""Generate the KiCad (v7+ format) project for the Duo S MIPI-DSI -> Tang Nano 9K adapter.
Run: python3 gen_project.py   (re-creates the .kicad_* files in this directory)

Receive network per lane (CK, D0, D1), see dphy_full_lp_network.png:
  split 2x50R + 100nF termination, AC-coupled HS to FPGA LVDS pair with 0.9V VCM bias,
  LP_P tap through 470R.  D0 LP goes straight to 1.8V pin 85; CK/D1 LP go through
  1.8V->3.3V input-translating buffers (SN74AUP1T17) to J5 pins 25/26."""
import json, uuid, os

NAME = "duos_dsi_to_tang9k"
HERE = os.path.dirname(os.path.abspath(__file__))
U = lambda: str(uuid.uuid4())
EFF = "(effects (font (size 1.27 1.27)))"
G = "GND"

# ---- connectivity ----------------------------------------------------------------
# J1: Duo S CON26A (odd pins left col, even right).  MIPI_TX pair0=D0, pair1=D1, pair2=CLK, pair3=D2, pair4=D3
J1 = ["DSI_D0_P", "DSI_D1_P", "DSI_D0_N", "DSI_D1_N", G, G, "DSI_CK_P", "DSI_D2_P", "DSI_CK_N", "DSI_D2_N",
      G, G, "DSI_D3_P", None, "DSI_D3_N", None, None, None, None, None, None, None, None, None, None, None]

def hdr24(m):
    l = [None] * 24
    for k, v in m.items():
        l[k - 1] = v
    return l
# J2 mirrors Tang Nano 9K header J6 (pin n = J6 pin n).  A half of each IOT pair = P (true), B = N.
J2 = hdr24({3: "LP_D0_P",                 # PIN85 IOT8B  (1.8V, single-ended)
            4: "HS_D1_P", 5: "HS_D1_N",   # PIN84/83 IOT10A/B
            6: "HS_D0_P", 7: "HS_D0_N",   # PIN82/81 IOT11A/B
            8: "HS_CK_P", 9: "HS_CK_N",   # PIN80/79 IOT12A/B
            23: G, 24: "+3V3"})
# J3 mirrors Tang Nano 9K header J5 (pin n = J5 pin n)
J3 = hdr24({5: "LP_CK_3V3",               # PIN25 IOB8A
            6: "LP_D1_3V3"})              # PIN26 IOB8B
ROW_PITCH = 17.78   # ASSUMED distance between the two Tang header rows (0.7in) - measure your board

# 2-pin parts: (kind, value, netA, netB)
PARTS = []
for x in ("CK", "D0", "D1"):
    PARTS += [("R", "50R", f"DSI_{x}_P", f"MID_{x}"), ("R", "50R", f"DSI_{x}_N", f"MID_{x}"),
              ("C", "100nF", f"MID_{x}", G),
              ("C", "100nF", f"DSI_{x}_P", f"HS_{x}_P"), ("C", "100nF", f"DSI_{x}_N", f"HS_{x}_N"),
              ("R", "10k", f"HS_{x}_P", "VCM"), ("R", "10k", f"HS_{x}_N", "VCM")]
PARTS += [("R", "470R", "DSI_D0_P", "LP_D0_P"),
          ("R", "470R", "DSI_CK_P", "LPIN_CK"), ("R", "470R", "DSI_D1_P", "LPIN_D1"),
          ("R", "27k", "+3V3", "VCM"), ("R", "10k", "VCM", G), ("C", "100nF", "VCM", G),
          ("C", "100nF", "+3V3", G), ("C", "100nF", "+3V3", G)]
# ICs: SN74AUP1T17 (SOT-23-5): 1 NC, 2 A, 3 GND, 4 Y, 5 VCC
ICS = [("U1", {2: "LPIN_CK", 3: G, 4: "LP_CK_3V3", 5: "+3V3"}),
       ("U2", {2: "LPIN_D1", 3: G, 4: "LP_D1_3V3", 5: "+3V3"})]

cnt = {"R": 0, "C": 0}
PL = []   # (ref, kind, value, a, b)
for kind, val, a, b in PARTS:
    cnt[kind] += 1
    PL.append((f"{kind}{cnt[kind]}", kind, val, a, b))

allnets = set()
for l in (J1, J2, J3):
    allnets |= {n for n in l if n}
for _, _, _, a, b in PL:
    allnets |= {a, b}
for _, m in ICS:
    allnets |= set(m.values())
NETS = ["GND"] + sorted(allnets - {"GND"})
NID = {n: i + 1 for i, n in enumerate(NETS)}

# ---- schematic ------------------------------------------------------------------
def conn_sym(n):
    pins = ""
    top = (n - 1) * 1.27
    for i in range(n):
        y = top - 2.54 * i
        pins += (f'(pin passive line (at -3.81 {y:.2f} 0) (length 2.54) '
                 f'(name "Pin_{i+1}" {EFF}) (number "{i+1}" {EFF}))\n')
    h = top + 1.27
    return (f'(symbol "Connector_Generic:Conn_01x{n:02d}" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)\n'
            f'(property "Reference" "J" (at 0 {h+2.54:.2f} 0) {EFF})\n'
            f'(property "Value" "Conn_01x{n:02d}" (at 0 {-h-2.54:.2f} 0) {EFF})\n'
            f'(property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))\n'
            f'(symbol "Conn_01x{n:02d}_1_1" (rectangle (start -1.27 {h:.2f}) (end 6.35 {-h:.2f}) '
            f'(stroke (width 0.254) (type default)) (fill (type background)))\n{pins})\n)')

R_SYM = f'''(symbol "Device:R" (pin_numbers hide) (pin_names (offset 0)) (in_bom yes) (on_board yes)
(property "Reference" "R" (at 2.032 0 90) {EFF})
(property "Value" "R" (at -2.032 0 90) {EFF})
(property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
(symbol "R_1_1" (rectangle (start -1.016 -2.54) (end 1.016 2.54) (stroke (width 0.254) (type default)) (fill (type none)))
(pin passive line (at 0 3.81 270) (length 1.27) (name "~" {EFF}) (number "1" {EFF}))
(pin passive line (at 0 -3.81 90) (length 1.27) (name "~" {EFF}) (number "2" {EFF})))
)'''
C_SYM = f'''(symbol "Device:C" (pin_numbers hide) (pin_names (offset 0.254)) (in_bom yes) (on_board yes)
(property "Reference" "C" (at 2.032 0 90) {EFF})
(property "Value" "C" (at -2.032 0 90) {EFF})
(property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
(symbol "C_1_1" (polyline (pts (xy -2.032 -0.762) (xy 2.032 -0.762)) (stroke (width 0.508) (type default)) (fill (type none)))
(polyline (pts (xy -2.032 0.762) (xy 2.032 0.762)) (stroke (width 0.508) (type default)) (fill (type none)))
(pin passive line (at 0 3.81 270) (length 3.048) (name "~" {EFF}) (number "1" {EFF}))
(pin passive line (at 0 -3.81 90) (length 3.048) (name "~" {EFF}) (number "2" {EFF})))
)'''
IC_SYM = f'''(symbol "Custom:SN74AUP1T17" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)
(property "Reference" "U" (at 0 8.5 0) {EFF})
(property "Value" "SN74AUP1T17" (at 0 -8.5 0) {EFF})
(property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
(symbol "SN74AUP1T17_1_1" (rectangle (start -5.08 -5.08) (end 5.08 5.08) (stroke (width 0.254) (type default)) (fill (type background)))
(pin passive line (at -7.62 2.54 0) (length 2.54) (name "NC" {EFF}) (number "1" {EFF}))
(pin passive line (at -7.62 0 0) (length 2.54) (name "A" {EFF}) (number "2" {EFF}))
(pin passive line (at 0 -7.62 90) (length 2.54) (name "GND" {EFF}) (number "3" {EFF}))
(pin passive line (at 7.62 0 180) (length 2.54) (name "Y" {EFF}) (number "4" {EFF}))
(pin passive line (at 0 7.62 270) (length 2.54) (name "VCC" {EFF}) (number "5" {EFF})))
)'''

root = U()
sch_items = []

def label(net, x, y, ang):
    j = " (justify right)" if ang == 180 else ""
    sch_items.append(f'(label "{net}" (at {x:.2f} {y:.2f} {ang}) (effects (font (size 1.27 1.27)){j}) (uuid {U()}))')

def inst(lib, ref, val, fp, x, y, npins):
    pins = "".join(f'(pin "{i+1}" (uuid {U()}))' for i in range(npins))
    sch_items.append(
        f'(symbol (lib_id "{lib}") (at {x} {y} 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid {U()})\n'
        f'(property "Reference" "{ref}" (at {x+3} {y-1.5} 0) (effects (font (size 1.27 1.27)) (justify left)))\n'
        f'(property "Value" "{val}" (at {x+3} {y+1.5} 0) (effects (font (size 1.27 1.27)) (justify left)))\n'
        f'(property "Footprint" "{fp}" (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))\n'
        f'{pins}\n(instances (project "{NAME}" (path "/{root}" (reference "{ref}") (unit 1)))))')

def connector(ref, val, fp, nets, x, y):
    n = len(nets)
    inst(f"Connector_Generic:Conn_01x{n:02d}", ref, val, fp, x, y, n)
    top = (n - 1) * 1.27
    for i, net in enumerate(nets):
        if net:
            label(net, x - 3.81, y - (top - 2.54 * i), 180)

HDR = "Connector_PinHeader_2.54mm:PinHeader_"
connector("J1", "CON26A_Duo_S", HDR + "2x13_P2.54mm_Vertical", J1, 40, 100)
connector("J2", "Tang_J6_(PIN85..79)", HDR + "1x24_P2.54mm_Vertical", J2, 40, 170)
connector("J3", "Tang_J5_(PIN25/26)", HDR + "1x24_P2.54mm_Vertical", J3, 40, 240)

COLS, X0, Y0, DX, DY = 5, 110, 40, 28, 24
FPN = {"R": "Resistor_SMD:R_0402_1005Metric", "C": "Capacitor_SMD:C_0402_1005Metric"}
for k, (ref, kind, val, a, b) in enumerate(PL):
    x, y = X0 + DX * (k % COLS), Y0 + DY * (k // COLS)
    inst(f"Device:{kind}", ref, val, FPN[kind], x, y, 2)
    label(a, x, y - 3.81, 90)
    label(b, x, y + 3.81, 270)
ICY = Y0 + DY * ((len(PL) + COLS - 1) // COLS) + 14
for k, (ref, m) in enumerate(ICS):
    x, y = X0 + 40 * k + 10, ICY
    inst("Custom:SN74AUP1T17", ref, "SN74AUP1T17", "Package_TO_SOT_SMD:SOT-23-5", x, y, 5)
    sch_items.append(f'(no_connect (at {x-7.62} {y-2.54}) (uuid {U()}))')
    label(m[2], x - 7.62, y, 180); label(m[4], x + 7.62, y, 0)
    label(m[5], x, y - 7.62, 90); label(m[3], x, y + 7.62, 270)
notes = ("Per lane (CK, D0, D1): 2x50R + 100nF split termination; HS AC-coupled (100nF) to the FPGA LVDS pair, biased to VCM 0.9V (27k/10k from 3V3).\\n"
         "LP: 470R tap from DSI_x_P. D0 -> FPGA pin 85 (1.8V). CK/D1 -> SN74AUP1T17 (3.3V supply) -> pins 25/26 (3.3V bank).\\n"
         "J2 = Tang header J6, J3 = Tang header J5 (pin n = pin n). J1 = Duo S CON26A. ROW_PITCH between J2/J3 is ASSUMED.")
sch_items.append(f'(text "{notes}" (at 110 20 0) (effects (font (size 1.5 1.5)) (justify left)) (uuid {U()}))')

sch = (f'(kicad_sch (version 20230121) (generator "gen_project.py") (uuid {root}) (paper "A3")\n'
       f'(lib_symbols\n{conn_sym(26)}\n{conn_sym(24)}\n{R_SYM}\n{C_SYM}\n{IC_SYM}\n)\n' + "\n".join(sch_items) +
       '\n(sheet_instances (path "/" (page "1"))))\n')

# ---- PCB ------------------------------------------------------------------------
def hdr_fp(ref, val, libname, nets, x, y, cols=1):
    n = len(nets)
    rows = (n + cols - 1) // cols
    pads = ""
    for i, net in enumerate(nets):
        shape = "rect" if i == 0 else "oval"
        px, py = 2.54 * (i % cols), 2.54 * (i // cols)
        nt = f' (net {NID[net]} "{net}")' if net else ""
        pads += (f'(pad "{i+1}" thru_hole {shape} (at {px:.2f} {py:.2f}) (size 1.7 1.7) (drill 1.0) '
                 f'(layers "*.Cu" "*.Mask"){nt})\n')
    return (f'(footprint "Connector_PinHeader_2.54mm:{libname}" (layer "F.Cu") (tstamp {U()}) (at {x} {y})\n'
            f'(property "Reference" "{ref}" (at 0 -2.5 0) (layer "F.SilkS") {EFF})\n'
            f'(property "Value" "{val}" (at 0 {2.54*rows+0.5:.2f} 0) (layer "F.Fab") {EFF})\n'
            f'(fp_rect (start -1.33 -1.33) (end {2.54*(cols-1)+1.33:.2f} {2.54*(rows-1)+1.33:.2f}) (stroke (width 0.12) (type default)) (fill none) (layer "F.SilkS"))\n'
            f'{pads})')

def two_pin_fp(ref, kind, val, a, b, x, y):
    lib = FPN[kind]
    return (f'(footprint "{lib}" (layer "F.Cu") (tstamp {U()}) (at {x} {y} 90)\n'
            f'(property "Reference" "{ref}" (at 0 -1.2 90) (layer "F.SilkS") {EFF})\n'
            f'(property "Value" "{val}" (at 0 1.2 90) (layer "F.Fab") {EFF})\n'
            f'(pad "1" smd roundrect (at -0.51 0) (size 0.54 0.64) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25) (net {NID[a]} "{a}"))\n'
            f'(pad "2" smd roundrect (at 0.51 0) (size 0.54 0.64) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25) (net {NID[b]} "{b}"))\n)')

def sot235_fp(ref, m, x, y):
    pos = {1: (-1.1375, -0.95), 2: (-1.1375, 0), 3: (-1.1375, 0.95), 4: (1.1375, 0.95), 5: (1.1375, -0.95)}
    pads = ""
    for p, (px, py) in pos.items():
        nt = f' (net {NID[m[p]]} "{m[p]}")' if p in m else ""
        pads += (f'(pad "{p}" smd roundrect (at {px} {py}) (size 1.325 0.6) (layers "F.Cu" "F.Paste" "F.Mask") '
                 f'(roundrect_rratio 0.25){nt})\n')
    return (f'(footprint "Package_TO_SOT_SMD:SOT-23-5" (layer "F.Cu") (tstamp {U()}) (at {x} {y})\n'
            f'(property "Reference" "{ref}" (at 0 -2.4 0) (layer "F.SilkS") {EFF})\n'
            f'(property "Value" "SN74AUP1T17" (at 0 2.4 0) (layer "F.Fab") {EFF})\n{pads})')

W, H = 66, 70
J2X = W - 6
J3X = J2X - ROW_PITCH
fps = [hdr_fp("J1", "CON26A_Duo_S", "PinHeader_2x13_P2.54mm_Vertical", J1, 5, 4.5, 2),
       hdr_fp("J2", "Tang_J6", "PinHeader_1x24_P2.54mm_Vertical", J2, J2X, 5),
       hdr_fp("J3", "Tang_J5", "PinHeader_1x24_P2.54mm_Vertical", J3, J3X, 5)]
PCOLS = 4
for k, (ref, kind, val, a, b) in enumerate(PL):
    fps.append(two_pin_fp(ref, kind, val, a, b, 14 + 5.5 * (k % PCOLS), 8 + 5 * (k // PCOLS)))
icy = 8 + 5 * ((len(PL) + PCOLS - 1) // PCOLS) + 3
for k, (ref, m) in enumerate(ICS):
    fps.append(sot235_fp(ref, m, 17 + 8 * k, icy))
layers = '''(0 "F.Cu" signal) (31 "B.Cu" signal) (32 "B.Adhes" user "B.Adhesive") (33 "F.Adhes" user "F.Adhesive")
(34 "B.Paste" user) (35 "F.Paste" user) (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen")
(38 "B.Mask" user) (39 "F.Mask" user) (44 "Edge.Cuts" user) (45 "Margin" user)
(46 "B.CrtYd" user "B.Courtyard") (47 "F.CrtYd" user "F.Courtyard") (48 "B.Fab" user) (49 "F.Fab" user)'''
netdecl = '(net 0 "")\n' + "\n".join(f'(net {i} "{n}")' for n, i in NID.items())
pcb = (f'(kicad_pcb (version 20221018) (generator "gen_project.py")\n'
       f'(general (thickness 0.8))\n(paper "A4")\n(layers\n{layers}\n)\n'
       f'(setup (pad_to_mask_clearance 0))\n{netdecl}\n' + "\n".join(fps) +
       f'\n(gr_rect (start 0 0) (end {W} {H}) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))\n'
       f'(gr_text "GND pour on B.Cu - route DSI_*/HS_* as 100R diff pairs on F.Cu" (at 25 {H-2}) (layer "F.SilkS") {EFF})\n'
       f'(zone (net {NID["GND"]}) (net_name "GND") (layer "B.Cu") (tstamp {U()}) (hatch edge 0.5)\n'
       f'(connect_pads (clearance 0.3)) (min_thickness 0.25) (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
       f'(polygon (pts (xy 0 0) (xy {W} 0) (xy {W} {H}) (xy 0 {H}))))\n)\n')

# ---- project --------------------------------------------------------------------
def nc(name, **kw):
    d = {"name": name, "clearance": 0.2, "track_width": 0.25, "via_diameter": 0.6, "via_drill": 0.3,
         "diff_pair_width": 0.25, "diff_pair_gap": 0.2, "diff_pair_via_gap": 0.25,
         "microvia_diameter": 0.3, "microvia_drill": 0.1, "wire_width": 6, "bus_width": 12,
         "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)"}
    d.update(kw)
    return d
pro = {"meta": {"filename": f"{NAME}.kicad_pro", "version": 1},
       "net_settings": {"classes": [nc("Default"), nc("MIPI_DPHY_100R", clearance=0.2)],
                        "meta": {"version": 3},
                        "netclass_patterns": [{"netclass": "MIPI_DPHY_100R", "pattern": "DSI_*"},
                                              {"netclass": "MIPI_DPHY_100R", "pattern": "HS_*"}]}}

for fn, txt in {f"{NAME}.kicad_sch": sch, f"{NAME}.kicad_pcb": pcb,
                f"{NAME}.kicad_pro": json.dumps(pro, indent=2)}.items():
    with open(os.path.join(HERE, fn), "w") as f:
        f.write(txt)
print("wrote", NAME, "parts:", len(PL), "ics:", len(ICS), "nets:", len(NETS))
