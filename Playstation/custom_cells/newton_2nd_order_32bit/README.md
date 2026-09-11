# Full-Custom Transistor-Level IC Design & Verification Library
## High-Performance Datapath Cells (Cadence Virtuoso & Spectre Flow)

This repository contains the full-custom transistor-level integrated circuit (IC) designs developed from scratch at the device, schematic, and layout level for high-speed datapath acceleration in SCL 180nm CMOS technology.

The repository follows the same standardized, modular structure as the project's synthesis and verification environments, adhering to industrial EDA directory conventions.

---

## 1. Directory Structure

```text
Playstation/custom_cells/newton_2nd_order_32bit/
├── env_virtuoso.sh               # Root environment initialization script
├── cds.lib                       # Root Cadence OpenAccess library definitions
├── Makefile                      # Push-button simulation & design cockpit
├── README.md                     # Directory documentation & architectural guide
├── CUSTOM_CELLS_SIGNOFF.md       # Top-level executive sign-off deliverable
├── GEMINI.md                     # Workspace formatting rules
├── .gitignore                    # Robust artifact ignore list
│
├── filelist/                     # Design manifests & netlist filelists
│   ├── cells.f                   # All 12 custom transistor cells & testbenches
│   ├── macros.f                  # Datapath macros (32-bit multipliers, adders, dividers)
│   └── behavioral.f              # SystemVerilog co-simulation models
│
├── scripts/                      # Cadence Virtuoso & Spectre automation scripts
│   ├── env_virtuoso.sh           # Core environment initialization script
│   ├── run_cell_sim.sh           # Single-cell Spectre simulation runner
│   ├── run_regression.sh         # Complete 12-cell regression test runner
│   ├── gen_report.py             # Automated metrics harvester & signoff generator
│   ├── import_schematic.sh       # SpiceIn automated schematic generator
│   ├── open_design.sh            # Virtuoso GUI launcher (schematic/layout)
│   └── view_wave.sh              # Cadence ViVA waveform viewer launcher
│
├── logs/                         # Spectre simulation & Virtuoso execution logs
│   ├── *_spectre.log
│   └── clean_labels_*.log
│
├── outputs/                      # Raw PSF waveform datasets (*.raw directories)
│   └── *.raw/
│
├── reports/                      # Automated simulation reports & sign-off audits
│   ├── *_report.txt              # Individual cell simulation reports
│   ├── custom_cells_summary.rpt  # Aggregated PPA & delay summary table
│   └── custom_cells_regression.rpt # Full regression execution audit
│
├── work/                         # Ephemeral runtime scratchpad (gitignored)
│
├── cells/                        # Full-Custom Transistor-Level Standard Cells
│   ├── arithmetic/               # booth_encoder, booth_selector, compressor_4to2, csa_3to2_slice, kogge_stone_cells
│   ├── divider/                  # cas_divider_slice, srt_radix4_stage
│   ├── memory/                   # bitcell_8t_2r1w, sram_6t_cell
│   └── specialized/              # dynamic_overflow_detect, fast_comparator, tg_mux
│
├── macros/                       # Multi-bit datapath modules
│   ├── booth_wallace_mul_32b/
│   ├── fused_3input_adder_32b/
│   ├── kogge_stone_adder_32b/
│   ├── radix4_srt_divider_32b/
│   ├── regfile_2r1w_16x32b/
│   └── sram_prog_mem_32x32b/
│
├── behavioral/                   # SystemVerilog behavioral models (for co-simulation)
├── config/                       # PDK & simulator technology config (models.spice, devmap.txt)
├── docs/                         # Technical theory & schematic documentation
└── oa_libs/                      # Persistent OpenAccess Cadence database (custom_cells_oa)
```

---

## 2. Push-Button Workflow (Makefile Targets)

All tasks are centralized through the top-level [Makefile](file:///home/user17/Desktop/ju_project/Playstation/custom_cells/Makefile):

| Command | Action |
| :--- | :--- |
| `make check` | Verify Cadence Virtuoso and Spectre binaries and license servers |
| `make sim CELL=<name>` | Run Spectre transient simulation for target cell |
| `make wave CELL=<name>` | Launch Cadence ViVA Waveform Viewer for the cell simulation dataset |
| `make regression` | Run automated simulation regression across all 12 cells and compile sign-off report |
| `make sim_all` | Alias for `make regression` |
| `make reports` | Parse simulation outputs and update `CUSTOM_CELLS_SIGNOFF.md` |
| `make schematic CELL=<name>` | Import SPICE netlist into Cadence Virtuoso OpenAccess schematic view |
| `make view_schematic CELL=<n>` | Open the schematic diagram directly in Virtuoso GUI |
| `make view_layout CELL=<name>` | Open the layout editor directly in Virtuoso GUI |
| `make virtuoso` | Launch Cadence Virtuoso Library Manager |
| `make clean` | Remove runtime logs, waveforms, temporary files & lockfiles (**preserves `oa_libs/`**) |

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

To initialize the environment:
```bash
source ./env_virtuoso.sh
```

To simulate any cell using Cadence Spectre:
```bash
make sim CELL=compressor_4to2
```

To run complete regression across all 12 custom cells:
```bash
make regression
```

The automated runner executes:
1. Assembles top-level simulation deck with PDK transistor model cards.
2. Invokes Cadence Spectre with 64-bit precision and PSF binary output.
3. Automatically parses transient steps, convergence metrics, and `.measure` timing delays.
4. Generates an automated verification report in `reports/<cell_name>_report.txt`.
5. Saves waveform database in `outputs/<cell_name>.raw` for inspection in Cadence ViVA (`make wave CELL=<cell_name>`).
6. Aggregates metrics and updates `CUSTOM_CELLS_SIGNOFF.md` and `reports/custom_cells_summary.rpt`.
