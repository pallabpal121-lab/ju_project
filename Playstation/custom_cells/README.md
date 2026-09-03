# Full-Custom Transistor-Level IC Design & Verification Library
## High-Performance Datapath Cells (Cadence Virtuoso & Spectre Flow)

This repository contains the full-custom transistor-level integrated circuit (IC) designs developed from scratch at the device, schematic, and layout level for high-speed datapath acceleration.

The repository is structured strictly for **custom transistor design, SPICE/Spectre simulation, and Virtuoso verification**, adhering to industrial EDA directory conventions.

---

## 1. Directory Structure

```
custom_cells/
├── cells/                               # Full-Custom Transistor-Level Standard Cells
│   ├── arithmetic/
│   │   ├── booth_encoder/               # Radix-4 Modified Booth Encoder
│   │   │   ├── schematic.spice          # Transistor netlist (W/L sizing)
│   │   │   └── tb_transient.spice       # Spectre transient simulation testbench
│   │   ├── booth_selector/              # Radix-4 Booth Partial Product Selector
│   │   │   ├── schematic.spice
│   │   │   └── tb_transient.spice
│   │   ├── compressor_4to2/             # 28T TG 4:2 Carry-Save Compressor
│   │   │   ├── schematic.spice
│   │   │   └── tb_transient.spice
│   │   ├── csa_3to2_slice/              # Fused 3:2 Carry-Save Slice Cell
│   │   │   ├── schematic.spice
│   │   │   └── tb_transient.spice
│   │   └── kogge_stone_cells/           # PG, Black (AOI21), Gray, Sum (XOR) cells
│   │       ├── schematic.spice
│   │       └── tb_transient.spice
│   │
│   ├── divider/
│   │   ├── cas_divider_slice/           # Controlled Add/Subtract (CAS) 1-bit slice
│   │   │   ├── schematic.spice
│   │   │   └── tb_transient.spice
│   │   └── srt_radix4_stage/            # Radix-4 SRT Division Stage with CSA
│   │       ├── schematic.spice
│   │       └── tb_transient.spice
│   │
│   ├── memory/
│   │   ├── bitcell_8t_2r1w/             # 8T Dual-Read Single-Write Bitcell & Sense Amp
│   │   │   ├── schematic.spice
│   │   │   ├── sense_amplifier.spice
│   │   │   └── tb_read_write.spice
│   │   └── sram_6t_cell/                # 6T Static RAM Bitcell
│   │       ├── schematic.spice
│   │       └── tb_snm.spice
│   │
│   └── specialized/
│       ├── dynamic_overflow_detect/     # Fast dynamic overflow detector
│       │   ├── schematic.spice
│       │   └── tb_transient.spice
│       ├── fast_comparator/             # Magnitude comparator bit-slice
│       │   ├── schematic.spice
│       │   └── tb_transient.spice
│       └── tg_mux/                      # Transmission Gate MUX (2:1 and 4:1)
│           ├── schematic.spice
│           └── tb_transient.spice
│
├── behavioral/                          # SystemVerilog behavioral models (for co-simulation)
│   ├── booth_encoder.sv
│   ├── compressor_4to2.sv
│   ├── cas_cell.sv
│   ├── dynamic_overflow_detect.sv
│   ├── fast_comparator_32b.sv
│   └── tg_mux.sv
│
├── macros/                              # Multi-bit datapath modules (reference)
│   ├── booth_wallace_mul_32b/
│   ├── fused_3input_adder_32b/
│   ├── kogge_stone_adder_32b/
│   ├── radix4_srt_divider_32b/
│   ├── regfile_2r1w_16x32b/
│   └── sram_prog_mem_32x32b/
│
├── oa_libs/                             # Persistent OpenAccess Database
│   └── custom_cells_oa/                 # Virtuoso cellviews (schematics, symbols, layouts)
│
├── config/                              # EDA & PDK Environment Configuration
│   ├── cds.lib                          # Virtuoso library definitions
│   ├── env_virtuoso.sh                  # Cadence & PDK environment variables
│   ├── devmap.txt                       # SPICE-to-Virtuoso device mapping
│   ├── models.spice                     # Core 1.8V transistor SPICE models
│   └── models.scs                       # Spectre include wrapper
│
├── scripts/                             # Cadence Automation Scripts
│   ├── import_schematic.sh              # SpiceIn automated schematic generator
│   ├── open_design.sh                   # Virtuoso GUI launcher (schematic/layout)
│   ├── run_cell_sim.sh                  # Spectre batch runner & measurement extractor
│   └── view_wave.sh                     # Cadence ViVA waveform viewer launcher
│
├── docs/                                # Technical Architecture & Theory
│   ├── CIRCUIT_SCHEMATICS.md            # Transistor topologies & node connections
│   ├── TRANSISTOR_SIZING_THEORY.md      # Logical effort, Euler paths, stick diagrams
│   ├── PPA_OPTIMIZATION_REPORT.md       # Custom cell vs standard-cell PPA benchmarks
│   └── SCL180_PDK_MIGRATION_GUIDE.md    # SCL 180nm PDK integration guide
│
├── Makefile                             # Push-button simulation & design cockpit
│
├── work/                                # Ephemeral run scratchpad (gitignored)
├── logs/                                # Spectre simulation logs (gitignored)
├── reports/                             # Timing & measurement reports (gitignored)
└── outputs/                             # Raw waveform datasets (.raw / PSF) (gitignored)
```

---

## 2. Push-Button Workflow (Makefile Targets)

All tasks are centralized through the top-level [Makefile](file:///home/user17/Desktop/ju_project/Playstation/custom_cells/Makefile):

| Command | Action |
| :--- | :--- |
| `make check` | Verify Cadence Virtuoso and Spectre binaries and license servers |
| `make sim CELL=<name>` | Run Spectre transient simulation & generate automated verification report |
| `make sim_all` | Run Spectre simulation regression across all 12 custom cells |
| `make wave CELL=<name>` | Launch Cadence ViVA Waveform Viewer for the cell simulation dataset |
| `make schematic CELL=<name>` | Import SPICE netlist into Cadence Virtuoso OpenAccess schematic view |
| `make view_schematic CELL=<n>` | Open the schematic diagram directly in Virtuoso GUI |
| `make view_layout CELL=<name>` | Open the layout editor directly in Virtuoso GUI |
| `make virtuoso` | Launch Cadence Virtuoso Library Manager |
| `make clean` | Remove runtime logs, waveforms, reports & lockfiles (**preserves `oa_libs/`**) |

---

## 3. Custom Cell Catalog

| Domain | Cell Name | Architecture / Topology | Key Characteristics |
| :--- | :--- | :--- | :--- |
| **Arithmetic** | `compressor_4to2` | 28T Transmission-Gate 4:2 Compressor | 3 gate delays per stage, zero glitching |
| **Arithmetic** | `booth_encoder` | Radix-4 Booth Encoder | Generates `single`, `double`, `neg` controls |
| **Arithmetic** | `booth_selector` | TG-based Partial Product Mux | Low-capacitance transmission gate selection |
| **Arithmetic** | `csa_3to2_slice` | Fused 3:2 Carry-Save Slice | Single-gate-delay 3-operand adder slice |
| **Arithmetic** | `kogge_stone_cells` | Prefix Operators (`PG`, `Black`, `Gray`, `Sum`) | High-speed parallel-prefix carry generation |
| **Divider** | `cas_divider_slice` | Controlled Add/Subtract Bit-Slice | Dynamic add/subtract operation with fast mux carry |
| **Divider** | `srt_radix4_stage` | Radix-4 SRT Stage Slice | Radix-4 quotient digit selection stage |
| **Memory** | `bitcell_8t_2r1w` | 8T Dual-Read Single-Write Bitcell + Sense Amp | Decoupled read buffer stacks, sub-0.3ns access |
| **Memory** | `sram_6t_cell` | 6T Static RAM Bitcell | High-density cross-coupled inverter storage cell |
| **Specialized**| `dynamic_overflow_detect` | 17-bit Multiplier Overflow Detector | Tree-based NOR/NAND fast zero/one detection |
| **Specialized**| `fast_comparator` | Magnitude Comparator Bit-Slice | Fast equality and greater-than bit slice |
| **Specialized**| `tg_mux` | Transmission Gate Mux (2:1 and 4:1) | Low insertion delay, rail-to-rail swing |

---

## 4. Simulation & Verification Flow

To simulate any cell using Cadence Spectre:
```bash
make sim CELL=compressor_4to2
```

The automated runner executes:
1. Assembles top-level simulation deck with PDK transistor model cards.
2. Invokes Cadence Spectre with 64-bit precision and PSF binary output.
3. Automatically parses transient steps, convergence metrics, and `.measure` timing delays.
4. Generates an automated verification report in `reports/<cell_name>_report.txt`.
5. Saves waveform database in `outputs/<cell_name>.raw` for inspection in Cadence ViVA (`make wave CELL=<cell_name>`).
