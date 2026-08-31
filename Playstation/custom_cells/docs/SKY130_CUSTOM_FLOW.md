# Custom Cell Design & Physical Flow Guide
## Target Process: SkyWater 130nm (`sky130_fd_sc_hd` / `sky130_fd_pr`)

This document outlines the end-to-end custom cell development flow, from schematic and transistor-level SPICE simulation to layout, DRC/LVS, characterization (.lib), and top-level RTL synthesis integration.

---

## 1. Toolchain & Environment Setup

| Stage | Open-Source EDA Tool | Commercial EDA Equivalent |
| :--- | :--- | :--- |
| **Schematic Capture** | Xschem / PyRTL | Cadence Virtuoso Schematic Editor |
| **SPICE Simulation** | ngspice / Xyce | Cadence Spectre / Synopsys HSPICE |
| **Physical Layout** | Magic VLSI / KLayout | Cadence Virtuoso Layout XL |
| **DRC / LVS Verification** | Magic / Netgen | Siemens Calibre / Cadence Pegasus |
| **Parasitic Extraction (PEX)** | Magic (`ext2spice`) | Calibre xRC / StarRC |
| **Liberty Characterization** | CharLib / OpenSTA | Cadence Liberate / Synopsys SiliconSmart |
| **RTL-to-GDS Synthesis** | OpenLane / Yosys | Synopsys Fusion Compiler / Cadence Innovus |

---

## 2. Step-by-Step Custom Cell Creation Flow

### Step 1: Schematic Entry & SPICE Modeling
1. Create schematic in Xschem (`.sch`) using SkyWater 130nm primitive transistors:
   - `sky130_fd_pr__nfet_01v8` (Standard 1.8V NMOS)
   - `sky130_fd_pr__pfet_01v8` (Standard 1.8V PMOS)
2. Export SPICE netlist (`.spice`) and include process corner models:
   ```spice
   .include "/path/to/skywater-pdk/libraries/sky130_fd_pr/latest/models/sky130.lib.spice" tt
   ```
3. Run transient simulations (`.tran 10p 20n`) to verify propagation delay ($t_{pd}$), rise/fall times ($t_r, t_f$), and power consumption.

### Step 2: Physical Layout Design
1. Open layout in Magic:
   ```bash
   magic -dnull -noconsole magic/compressor_4to2.mag
   ```
2. Adhere to Sky130 Standard Cell Grid:
   - Height: $2.72 \, \mu m$ (HD library pitch)
   - Power Rails: $V_{DD}$ at top rail, $V_{SS}$ at bottom rail on Metal 1 ($0.48 \, \mu m$ width).
   - Gate Poly pitch: $0.46 \, \mu m$.
   - Pins: Placed on routing tracks (Metal 1 / Metal 2).

### Step 3: DRC & LVS Verification
1. **DRC (Design Rule Checking)**:
   ```tcl
   drc check
   drc why
   ```
2. **LVS (Layout vs Schematic)**:
   - Extract layout netlist:
     ```tcl
     extract all
     ext2spice lvs
     ext2spice
     ```
   - Run Netgen LVS:
     ```bash
     netgen -batch lvs compressor_4to2.spice compressor_4to2.ext.spice sky130A_setup.tcl lvs.out
     ```

### Step 4: Timing & Power Characterization (.lib Generation)
1. Run automated characterization over PVT corners:
   - Typical: $1.8\text{V}, 25^\circ\text{C}, \text{TT}$
   - Slow (Worst): $1.6\text{V}, 125^\circ\text{C}, \text{SS}$
   - Fast (Best): $1.98\text{V}, -40^\circ\text{C}, \text{FF}$
2. Generate Liberty timing table (`.lib`) containing:
   - Input pin capacitance ($C_{in}$)
   - Non-linear delay tables (NLDM) indexed by input transition time and output load capacitance ($C_L$).

### Step 5: RTL / Synthesis Integration
Include the custom macro `.lib` and `.gds` / `.lef` during synthesis with Yosys/OpenLane:
```tcl
set ::env(EXTRA_LEFS) [list "$::env(DESIGN_DIR)/custom_cells/04_specialized_logic_cells/compressor_4to2/lef/compressor_4to2.lef"]
set ::env(EXTRA_LIBS) [list "$::env(DESIGN_DIR)/custom_cells/04_specialized_logic_cells/compressor_4to2/lib/compressor_4to2.lib"]
```
