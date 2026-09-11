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
Playstation/synthesis/newton_2nd_order_32bit/
├── env_dc.sh                     # Shell environment initialization script
├── Makefile                      # Top-level GNU Makefile for flow automation
├── README.md                     # Directory and flow documentation
├── SYNTHESIS_SIGNOFF.md          # Top-level executive sign-off summary
├── .synopsys_dc.setup            # DC startup file (search paths, WORK lib, target/link libs)
├── .gitignore                    # Ignore temporary caches and logs
├── constraints/                  # Timing, environmental, and exception constraints
│   ├── newton_2nd_order_top.sdc  # Primary SDC timing constraints (100 MHz default)
│   ├── design_env.sdc            # Environmental constraints (operating condition, loads, drivers)
│   └── timing_exceptions.sdc     # False path and multicycle timing exceptions
├── filelist/
│   ├── rtl.f                     # SystemVerilog source list with include directories
│   └── rtl_syn.tcl               # Tcl file list for analyze/elaborate
├── scripts/                      # Modular synthesis automation scripts
│   ├── apply_constraints.tcl     # SDC sourcing and cost function goals
│   ├── common_setup.tcl          # Shared parameters (clock, frequency, node selection)
│   ├── compile_design.tcl        # Optimization engine (compile_ultra / compile)
│   ├── dc_setup.tcl              # Target library selection and SVF recording
│   ├── export_outputs.tcl        # Netlist, SDC, SDF, and DDC file exports
│   ├── gen_report.py             # Python QoR extractor & sign-off markdown generator
│   ├── generate_reports.tcl      # Comprehensive QoR, area, timing, power reports
│   ├── read_design.tcl           # Analyze, elaborate, link, and check_design
│   ├── run_check.sh              # Quick linting wrapper script
│   ├── run_dc.sh                 # Command-line synthesis launcher
│   ├── run_gui.sh                # Design Vision GUI launcher
│   └── run_synthesis.tcl         # Master synthesis automation execution script
├── logs/                         # Synthesis execution and command logs
│   ├── check_design.log
│   ├── command.log
│   └── synthesis.log
├── reports/                      # Categorized sign-off reports
│   ├── qor.rpt                   # Top-level Quality of Results summary
│   ├── timing/                   # Setup/hold timing and constraint checks
│   │   ├── check_timing.rpt
│   │   ├── timing_hold_min.rpt
│   │   └── timing_setup_max.rpt
│   ├── area/                     # Hierarchical cell area and DesignWare resources
│   │   ├── area_hier.rpt
│   │   └── resources.rpt
│   ├── power/                    # Power dissipation and clock gating
│   │   ├── clock_gating.rpt
│   │   └── power.rpt
│   └── checks/                   # Linting, constraint violations, and standard cells
│       ├── check_design.rpt
│       ├── constraint_violators.rpt
│       └── references.rpt
├── outputs/                      # Categorized backend deliverables
│   ├── netlist/                  # Gate-level structural Verilog netlist (.netlist.v)
│   ├── constraints/              # Synthesized SDC constraints (.sdc)
│   ├── delays/                   # Back-annotation delay model (.sdf)
│   └── db/                       # Formal verification (.svf) and binary databases (.ddc)
└── work/                         # Tool internal scratch, WORK library, and ALIB cache
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
