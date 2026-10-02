#!/usr/bin/env python3
"""Generate the KiCad (v7+ format) project for the Duo S MIPI-DSI -> Tang Nano 9K adapter.
Run: python3 gen_project.py   (re-creates the .kicad_* files in this directory)"""
import json, uuid, os

NAME = "duos_dsi_to_tang9k"
HERE = os.path.dirname(os.path.abspath(__file__))
U = lambda: str(uuid.uuid4())
EFF = "(effects (font (size 1.27 1.27)))"

# ---- connectivity ----------------------------------------------------------------
# J1: Duo S 26-pin header (CON26A, from the Duo S schematic). Odd pins left column, even pins right.
# SG2000 MIPI_TX pairs 0..4 -> lane_id {D0, D1, CLK, D2, D3} (see cvi_mpi/component/panel/cv181x/dsi_milkv_8hd*.h)
G = "GND"
J1 = ["DSI_D0_P", "DSI_D1_P", "DSI_D0_N", "DSI_D1_N", G, G, "DSI_CK_P", "DSI_D2_P", "DSI_CK_N", "DSI_D2_N",
      G, G, "DSI_D3_P", None, "DSI_D3_N", None, None, None, None, None, None, None, None, None, None, None]
# J2: Tang Nano 9K side, FPGA pins 79..85 (+GND). Order of pins on the header is a PLACEHOLDER.
J2 = ["DSI_CK_P", "DSI_CK_N", "DSI_D0_P", "DSI_D0_N", "DSI_D1_P", "DSI_D1_N", "LP_D0P", "GND"]
J2_FPGA = ["79", "80", "81", "82", "83", "84", "85", "GND"]
# (ref, value, netA, netB, dnp)
RES = [("R1", "100R", "DSI_CK_P", "DSI_CK_N", False),
       ("R2", "100R", "DSI_D0_P", "DSI_D0_N", False),
       ("R3", "100R", "DSI_D1_P", "DSI_D1_N", False),
       ("R4", "470R", "DSI_D0_P", "LP_D0P", True)]
NETS = ["GND"] + sorted({n for n in J1 + J2 if n and n != "GND"})
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

root = U()
sch_items = []

def label(net, x, y, ang):
    j = " (justify right)" if ang == 180 else ""
    sch_items.append(f'(label "{net}" (at {x:.2f} {y:.2f} {ang}) (effects (font (size 1.27 1.27)){j}) (uuid {U()}))')

def inst(lib, ref, val, fp, x, y, npins, dnp=False):
    pins = "".join(f'(pin "{i+1}" (uuid {U()}))' for i in range(npins))
    sch_items.append(
        f'(symbol (lib_id "{lib}") (at {x} {y} 0) (unit 1) (in_bom yes) (on_board yes) (dnp {"yes" if dnp else "no"}) (uuid {U()})\n'
        f'(property "Reference" "{ref}" (at {x} {y-3} 0) {EFF})\n'
        f'(property "Value" "{val}" (at {x} {y+3} 0) {EFF})\n'
        f'(property "Footprint" "{fp}" (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))\n'
        f'{pins}\n(instances (project "{NAME}" (path "/{root}" (reference "{ref}") (unit 1)))))')

def connector(ref, val, fp, nets, x, y):
    n = len(nets)
    inst(f"Connector_Generic:Conn_01x{n:02d}", ref, val, fp, x, y, n)
    top = (n - 1) * 1.27
    for i, net in enumerate(nets):
        if net:
            label(net, x - 3.81, y - (top - 2.54 * i), 180)

connector("J1", "CON26A_Duo_S", "Connector_PinHeader_2.54mm:PinHeader_2x13_P2.54mm_Vertical", J1, 60, 90)
connector("J2", "Tang_Nano_9K", "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical", J2, 60, 170)
for k, (ref, val, a, b, dnp) in enumerate(RES):
    x, y = 140 + 25 * k, 100
    inst("Device:R", ref, val, "Resistor_SMD:R_0402_1005Metric", x, y, 2, dnp)
    label(a, x, y - 3.81, 90)
    label(b, x, y + 3.81, 270)
sch_items.append(f'(text "100R across each P/N pair is placed at the FPGA end (HS termination).\\nR4 (DNP) taps D0_P for LP-state sensing on FPGA pin 85.\\nJ1 = Duo S CON26A (MIPI_TX pair0=D0, pair1=D1, pair2=CLK). J2 pin order is a PLACEHOLDER." (at 140 70 0) (effects (font (size 1.5 1.5)) (justify left)) (uuid {U()}))')
sch_items.append(f'(text "FPGA pin 79..85 -> J2 pin 1..7 :  {", ".join(J2_FPGA[:7])}" (at 140 78 0) (effects (font (size 1.27 1.27)) (justify left)) (uuid {U()}))')

sch = (f'(kicad_sch (version 20230121) (generator "gen_project.py") (uuid {root}) (paper "A4")\n'
       f'(lib_symbols\n{conn_sym(26)}\n{conn_sym(8)}\n{R_SYM}\n)\n' + "\n".join(sch_items) +
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

def res_fp(ref, val, a, b, x, y, rot):
    return (f'(footprint "Resistor_SMD:R_0402_1005Metric" (layer "F.Cu") (tstamp {U()}) (at {x} {y} {rot})\n'
            f'(property "Reference" "{ref}" (at 0 -1.2 {rot}) (layer "F.SilkS") {EFF})\n'
            f'(property "Value" "{val}" (at 0 1.2 {rot}) (layer "F.Fab") {EFF})\n'
            f'(pad "1" smd roundrect (at -0.51 0) (size 0.54 0.64) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25) (net {NID[a]} "{a}"))\n'
            f'(pad "2" smd roundrect (at 0.51 0) (size 0.54 0.64) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25) (net {NID[b]} "{b}"))\n)')

W, H = 50, 40
fps = [hdr_fp("J1", "CON26A_Duo_S", "PinHeader_2x13_P2.54mm_Vertical", J1, 5, 4.5, 2),
       hdr_fp("J2", "Tang_Nano_9K", "PinHeader_1x08_P2.54mm_Vertical", J2, W - 5, 11)]
for k, (ref, val, a, b, dnp) in enumerate(RES):
    fps.append(res_fp(ref, val, a, b, 30, 10 + 5 * k, 90))
layers = '''(0 "F.Cu" signal) (31 "B.Cu" signal) (32 "B.Adhes" user "B.Adhesive") (33 "F.Adhes" user "F.Adhesive")
(34 "B.Paste" user) (35 "F.Paste" user) (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen")
(38 "B.Mask" user) (39 "F.Mask" user) (44 "Edge.Cuts" user) (45 "Margin" user)
(46 "B.CrtYd" user "B.Courtyard") (47 "F.CrtYd" user "F.Courtyard") (48 "B.Fab" user) (49 "F.Fab" user)'''
netdecl = '(net 0 "")\n' + "\n".join(f'(net {i} "{n}")' for n, i in NID.items())
pcb = (f'(kicad_pcb (version 20221018) (generator "gen_project.py")\n'
       f'(general (thickness 0.8))\n(paper "A4")\n(layers\n{layers}\n)\n'
       f'(setup (pad_to_mask_clearance 0))\n{netdecl}\n' + "\n".join(fps) +
       f'\n(gr_rect (start 0 0) (end {W} {H}) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))\n'
       f'(gr_text "GND pour on B.Cu - route DSI_* as 100R diff pairs on F.Cu" (at 25 {H-3}) (layer "F.SilkS") {EFF})\n'
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
                        "netclass_patterns": [{"netclass": "MIPI_DPHY_100R", "pattern": "DSI_*"}]}}

for fn, txt in {f"{NAME}.kicad_sch": sch, f"{NAME}.kicad_pcb": pcb,
                f"{NAME}.kicad_pro": json.dumps(pro, indent=2)}.items():
    with open(os.path.join(HERE, fn), "w") as f:
        f.write(txt)
print("wrote", NAME)
