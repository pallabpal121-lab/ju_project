# ASIC Synthesis Environment: Universal Multivariable Newton BCD Accelerator

Production-grade ASIC synthesis environment for the **Universal Multivariable Newton Block Coordinate Descent (BCD) Accelerator** (`newton_bcd_axi_lite`) targeting the **Semi-Conductor Laboratory (SCL) 180nm CMOS standard cell library** using **Synopsys Design Compiler (`dc_shell`)**.

---

## 1. Directory Structure

```
synthesis/newton_multivar_32bit/
├── Makefile                               # Standardized execution targets (synth, report, check, clean)
├── .synopsys_dc.setup                     # Synopsys DC initialization, library search paths & aliases
├── README.md                              # Technical documentation and execution guide
├── filelist/
│   └── rtl_synth.f                        # Ordered RTL source filelist with +incdir+ directives
├── constraints/
│   └── newton_multivar_scl180.sdc         # Production SDC 2.1 timing and electrical constraints
├── scripts/
│   ├── env_synth.sh                       # Environment configuration (tools, licenses, PDK paths)
│   ├── run_dc.tcl                         # Master batch synthesis and reporting Tcl script
│   └── gen_summary_report.py              # Automated QoR metric parser and sign-off report generator
├── logs/                                  # Compilation and synthesis log files
├── reports/                               # Timing, area, power, clock gating, and lint reports
└── outputs/                               # Gate-level netlists (.v), SDC (.sdc), SDF (.sdf), DDC (.ddc)
```

---

## 2. Technology & Operating Conditions

- **Target Technology**: SCL 180nm (0.18 um CMOS) Standard Cell Library
- **Nominal Core Voltage**: V_DD = 1.8 V
- **PVT Corners**:
  - **Worst-Case Setup (SS)**: 1.62 V, 125 C (`tsl18fs120_scl_ss.db`) -> Invoked with `CORNER=slow`
  - **Best-Case Hold (FF)**: 1.98 V, 0 C (`tsl18fs120_scl_ff.db`) -> Invoked with `CORNER=fast`
  - **Typical (TT)**: 1.80 V, 25 C (`tsl18fs120_typ.db`) -> Default corner
- **Standard Cell Driving Model**: BUF_X4 (or library equivalents `buffd4`/`bufbd4`/`inv0d4`)
- **Output Load Capacitance**: C_load = 30 fF (0.030 pF) per output pin

---

## 3. Timing Constraints Summary

The primary timing constraints are formalized in `constraints/newton_multivar_scl180.sdc`:
- **Primary Clock**: `s_axi_aclk` @ 50 MHz (period = 20.0 ns, 50% duty cycle)
- **Clock Uncertainty**: 0.400 ns setup uncertainty, 0.200 ns hold uncertainty
- **Clock Transition**: 0.200 ns slew
- **I/O Delay Budget**:
  - Input delay: 4.0 ns (20% of cycle) referenced to `s_axi_aclk` on all inputs except clocks
  - Output delay: 4.0 ns (20% of cycle) referenced to `s_axi_aclk` on all outputs (including `irq_done`)
- **Timing Exceptions**:
  - False path declared on asynchronous active-low reset: `set_false_path -from [get_ports s_axi_aresetn]`
- **Design Rule Constraints**:
  - Max Fanout = 16
  - Max Transition = 0.500 ns

---

## 4. Synthesis Execution Guide

### Quick Start
To run the full automated synthesis, QoR report generation, and sign-off check in one step:
```bash
make all
```

### Individual Makefile Targets

| Target | Description |
| :--- | :--- |
| `make synth` | Runs batch synthesis in `dc_shell`, compiles with `compile_ultra -gate_clock -retime`, and logs to `logs/synth.log` |
| `make report` | Runs `scripts/gen_summary_report.py`, extracts WNS, TNS, cell area, kGE, and power, and writes `SYNTHESIS_SIGNOFF.md` |
| `make check` | Validates sign-off criteria: WNS >= 0.0 ns (zero setup violations), zero latches, and presence of all output deliverables |
| `make check_design` | Quick RTL analyze, elaborate, link, and lint check without full compilation |
| `make gui` | Launches Synopsys Design Vision graphical user interface |
| `make clean` | Cleans temporary synthesis scratch files, logs, and cache (preserves reports and netlists) |
| `make distclean` | Removes all generated outputs, reports, logs, and sign-off documents |

### Configurable Parameter Overrides
You can customize clock periods or PVT corners directly via Makefile variables:
```bash
# Synthesize at 60 MHz (16.67 ns period) at Worst-Case Slow corner (SS, 1.62 V, 125 C)
make synth CLK_PERIOD=16.67 CORNER=slow

# Synthesize at Best-Case Fast corner (FF, 1.98 V, 0 C)
make synth CORNER=fast
```

---

## 5. Deliverables & Sign-Off Criteria

Upon successful execution, the following production deliverables are generated in `outputs/`:
- **Gate-Level Netlist**: `outputs/newton_bcd_axi_lite_synth.v` (for Place & Route in ICC2 and STA)
- **Gate-Level Constraints**: `outputs/newton_bcd_axi_lite_synth.sdc`
- **Standard Delay Format**: `outputs/newton_bcd_axi_lite_synth.sdf` (for dynamic gate-level simulation)
- **Synopsys Mapped Database**: `outputs/newton_bcd_axi_lite_synth_mapped.ddc`
- **Formal Verification SVF**: `outputs/newton_bcd_axi_lite.svf` (for equivalence checking in Formality)
