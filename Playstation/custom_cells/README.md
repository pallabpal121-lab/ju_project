# Full-Custom Transistor-Level IC Design Library
## Universal Newton 2nd-Order 32-Bit Accelerator

This directory contains full-custom transistor-level integrated circuit (IC) designs developed from scratch at the device, schematic, and layout level (not synthesized standard cells).

---

## 1. Directory Structure

```
custom_cells/
├── 01_arithmetic_datapath/
│   ├── compressor_4to2/           # 28T Transmission-Gate 4:2 Compressor
│   │   ├── schematic.spice        # Transistor netlist with W/L sizing
│   │   ├── tb_transient.spice     # SPICE transient analysis & delay testbench
│   │   └── DESIGN_GUIDE.md        # Euler paths, transistor sizing & stick diagrams
│   │
│   ├── booth_selector_cell/       # Radix-4 Modified Booth PP Generator Cell
│   │   ├── schematic.spice
│   │   ├── tb_transient.spice
│   │   └── DESIGN_GUIDE.md
│   │
│   ├── kogge_stone_prefix_cells/  # Prefix Operators (PG, Black, Gray, Sum Cells)
│   │   ├── black_cell.spice       # Transistor-level AOI21 prefix dot-operator
│   │   ├── gray_cell.spice
│   │   └── sum_cell.spice
│   │
│   └── fused_3input_csa_cell/     # Fused 3:2 Carry-Save Slice for 2nd Derivatives
│       ├── schematic.spice
│       └── tb_transient.spice
│
├── 02_division_acceleration/
│   ├── cas_divider_slice/         # Controlled Add/Subtract (CAS) 1-bit Slice
│   │   ├── schematic.spice
│   │   └── tb_transient.spice
│   │
│   └── srt_radix4_stage/          # Radix-4 SRT Division Stage with CSA Accumulator
│       ├── schematic.spice
│       └── DESIGN_GUIDE.md
│
├── 03_memory_storage_cells/
│   ├── bitcell_8t_2r1w/           # 8T Dual-Read Single-Write Register File Bitcell
│   │   ├── schematic.spice        # Cross-coupled inverters + decoupled read buffers
│   │   ├── sense_amplifier.spice  # Voltage-latch differential sense amp
│   │   └── tb_read_write.spice    # Read/write margin & timing testbench
│   │
│   └── sram_6t_prog_cell/         # Dense 6T Static RAM Bitcell for Microcode Storage
│       ├── schematic.spice
│       └── tb_snm.spice           # Static Noise Margin (SNM) butterfly curve test
│
└── docs/
    ├── TRANSISTOR_SIZING_THEORY.md# Logical Effort, W/L ratios, Euler paths
    └── CIRCUIT_SCHEMATICS.md      # Transistor-level schematic diagrams
```

---

## 2. Why Full-Custom Transistor Design for this RTL?

1. **q16_divider.sv (48-cycle bottleneck)**:
   - RTL does sequential 32-bit subtractions each cycle.
   - **Custom Design**: Radix-4 SRT cell with redundant Carry-Save Accumulator (CSA). Cuts cycle count to 24 cycles and removes the 32-bit carry-propagate delay from the critical clock loop!

2. **q16_alu.sv 32x32 Multiplication (Fmax bottleneck)**:
   - Synthesis infers large, high-power gate trees with significant glitching.
   - **Custom Design**: 28-transistor Transmission-Gate 4:2 compressors and Radix-4 Booth selectors. Reduces XOR critical path delay to 3 gate delays per stage with zero glitch propagation.

3. **dfg_equation_engine.sv Register File (Area bottleneck)**:
   - RTL uses 16 x 32 D-flip-flops + wide 16:1 mux trees (~16,000 transistors).
   - **Custom Design**: 8T Static RAM Bitcell with precharged bitlines (~4,500 transistors, 70% area reduction, sub-0.3ns read access).

4. **derivative_engine.sv Curvature Unit (f_+ - 2*f_0 + f_-)**:
   - RTL requires two serial adder/subtractor operations.
   - **Custom Design**: Fused 3:2 Carry-Save slice cell computing 3-operand curvature in a single gate delay.
