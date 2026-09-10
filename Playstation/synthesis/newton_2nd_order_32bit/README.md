# Industry-Standard Synopsys Design Compiler Synthesis Environment

This directory provides a production-grade, automated Synopsys Design Compiler (DC) synthesis flow for the **Universal Newton 2nd-Order Accelerator (32-bit Q16.16)** (`newton_2nd_order_top`).

The flow adheres to the Synopsys Reference Methodology (RM) and implements all concepts from the ASIC synthesis curriculum:
- Library definitions (.lib, .db, target, link, synthetic DesignWare)
- Process and operating conditions (Typical, Slow, Fast)
- Reading design files (SystemVerilog analyze & elaborate)
- Linking and design rule linting (`check_design`)
- Environmental constraints (Driving cells, pin loads, wire load models)
- Optimization constraints (Clock period, latency, uncertainty, transitions, IO delays)
- Timing exceptions (False paths on asynchronous resets and static microcode inputs)
- High-effort logic compilation and optimization
- Quality of Results (QoR), timing, area, and power sign-off reporting
- Gate-level netlist, SDC, SDF, and DDC deliverable generation

---

## Directory Layout

```text
Playstation/synthesis/
├── .synopsys_dc.setup            # DC startup file (search paths, target/link/synthetic libs)
├── env_dc.sh                     # Shell environment initialization script
├── Makefile                      # Top-level GNU Makefile for flow automation
├── README.md                     # Directory and flow documentation
├── filelist/
│   ├── rtl.f                     # SystemVerilog source list with include directories
│   └── rtl_syn.tcl               # Tcl file list for analyze/elaborate
├── constraints/
│   ├── newton_2nd_order_top.sdc  # Primary SDC timing constraints (100 MHz default)
│   ├── design_env.sdc            # Environmental constraints (operating condition, loads, drivers)
│   └── timing_exceptions.sdc     # False path and multicycle timing exceptions
├── scripts/
│   ├── common_setup.tcl          # Shared parameters (clock, frequency, node selection)
│   ├── dc_setup.tcl              # Target library selection and SVF recording
│   ├── read_design.tcl           # Analyze, elaborate, link, and check_design
│   ├── apply_constraints.tcl     # SDC sourcing and cost function goals
│   ├── compile_design.tcl        # Optimization engine (compile_ultra / compile)
│   ├── generate_reports.tcl      # Comprehensive QoR, area, timing, power reports
│   ├── export_outputs.tcl        # Netlist, SDC, SDF, and DDC file exports
│   └── run_synthesis.tcl         # Master synthesis automation execution script
├── reports/                      # Generated synthesis reports (.rpt)
├── outputs/                      # Mapped netlist (.v), SDC, SDF, and DDC database
├── logs/                         # Synthesis execution logs
└── work/                         # Temporary working directory and ALIB cache
```

---

## Quick Start

### 1. Source the Environment
```bash
source env_dc.sh
```

### 2. Check Design and Elaboration Only
```bash
make check_design
```

### 3. Run Full Batch Synthesis
```bash
make syn
```

### 4. Inspect Quality of Results (QoR) Summary
```bash
make reports
```

### 5. Launch Design Vision GUI
```bash
make gui
```

---

## Configuration Options

You can override synthesis parameters directly from the command line:

- **Target Clock Period**:
  ```bash
  make syn CLK_PERIOD=5.0     # 200 MHz
  make syn CLK_PERIOD=10.0    # 100 MHz (Default)
  ```

- **Technology Node & Library**:
  ```bash
  make syn TECH_NODE=SCL180 CORNER=typical   # SCL 180nm Commercial (Default)
  make syn TECH_NODE=SCL180 CORNER=slow      # SCL 180nm Worst-Case Slow
  make syn TECH_NODE=NANGATE45               # Nangate 45nm Open Cell Library
  ```

---

## Deliverables Generated

Upon successful completion of `make syn`, the following deliverables are populated in `outputs/`:
- `newton_2nd_order_top.netlist.v`: Mapped structural gate-level Verilog netlist for IC Compiler II (ICC2) Place & Route.
- `newton_2nd_order_top.sdc`: Post-synthesis timing constraints for PrimeTime STA and ICC2 APR.
- `newton_2nd_order_top.sdf`: Standard Delay Format back-annotation file for gate-level dynamic simulation with VCS.
- `newton_2nd_order_top_mapped.ddc`: Mapped Synopsys hierarchical database.
- `newton_2nd_order_top.svf`: Formality setup verification file for formal equivalence checking.
