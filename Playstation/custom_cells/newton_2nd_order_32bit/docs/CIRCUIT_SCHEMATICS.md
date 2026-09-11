# Transistor-Level Circuit Schematics & Topologies
## Universal Newton 2nd-Order 32-Bit Accelerator

This document details the transistor-level schematic topologies, device sizing ratios (W/L), and layout architectures for full-custom silicon implementation.

---

## 1. 28-Transistor Transmission-Gate 4:2 Carry-Save Compressor

### Functional Specification
- **Inputs**: X1, X2, X3, X4, CIN
- **Outputs**: SUM, CARRY, COUT
- **Equation**: X1 + X2 + X3 + X4 + CIN = SUM + 2 × (CARRY + COUT)

### Transistor Schematic Structure

```
         X1, X2                   X3, X4
           │                        │
     ┌─────┴─────┐            ┌─────┴─────┐
     │ 6T TG XOR │            │ 6T TG XOR │
     └─────┬─────┘            └─────┬─────┘
          X12                      X34
           │                        │
           └───────────┬────────────┘
                       │
                 ┌─────┴─────┐
                 │ 6T TG XOR │
                 └─────┬─────┘
                     X1234
                       │
        ┌──────────────┴──────────────┐
        │                             │
  ┌─────┴─────┐                 ┌─────┴─────┐
  │ 6T TG XOR │ (with CIN)      │ 4T TG MUX │ (CIN vs X4)
  └─────┬─────┘                 └─────┬─────┘
       SUM                         CARRY
```

### Transistor Sizing (Sky130 1.8V Process)
- PMOS TG pass transistors: W = 0.8 um, L = 0.15 um
- NMOS TG pass transistors: W = 0.5 um, L = 0.15 um
- Complementary Input Inverters: PMOS W = 1.0 um, NMOS W = 0.5 um

---

## 2. Radix-4 Modified Booth PP Selector Bit-Slice

### Functional Specification
Selects between 0, +1X, +2X, -1X, -2X multiplicand based on Booth control signals (single, double, neg).

### Transistor Schematic Structure

```
                  X_curr (1X)    X_prev (2X)
                      │              │
                      └───┬──────┬───┘
                          │      │
                      ┌───┴──────┴───┐
                      │  4T TG MUX   │ (Controlled by single/double)
                      └──────┬───────┘
                            MAG
                             │
                        ┌────┴────┐
                        │ 6T XOR  │ (Controlled by NEG)
                        └────┬────┘
                           PP_OUT
```

---

## 3. 8T Dual-Read Single-Write (2R1W) Register Bitcell

### Functional Specification
- Dedicated storage cell for the 16-register file (r0 to r15) in `dfg_equation_engine.sv`.
- Provides simultaneous read access on Port A (r[src_a]) and Port B (r[src_b]) without read disturbance.

### Transistor Schematic

```
               VDD                 VDD
                │                   │
             ┌──┴──┐             ┌──┴──┐
             │ MP1 │             │ MP2 │
             └──┬──┘             └──┬──┘
      BL_W ──┤  ├─── Q ───┬─── QB ──┤  ├── BLB_W
    (via MNW1)  │         │         │   (via MNW2)
             ┌──┴──┐      │      ┌──┴──┐
             │ MN1 │      │      │ MN2 │
             └──┬──┘      │      └──┬──┘
               VSS        │        VSS
                          │
         ┌────────────────┴────────────────┐
         │                                 │
     (Port A)                          (Port B)
     WL_RA ──[ MNA1 ]                  WL_RB ──[ MNB1 ]
                │                                 │
        Q ────[ MNA2 ]                     Q ────[ MNB2 ]
                │                                 │
               VSS                               VSS
         (Discharges BL_RA)                (Discharges BL_RB)
```

### Transistor Sizing Ratios for Stability
- **Cell Ratio (CR)**: W_MN1 / W_MNW1 = 1.0 um / 0.6 um = 1.67 (Guarantees write-ability and read stability)
- **Pull-Up Ratio (PR)**: W_MP1 / W_MNW1 = 0.5 um / 0.6 um = 0.83
- **Read Buffer Transistors**: W_MNA1 = W_MNA2 = 0.8 um, L = 0.15 um

---

## 4. Controlled Add/Subtract (CAS) Divider Slice

### Functional Specification
- Basic cell for Radix-4 SRT division stages.
- When CTRL = 0: performs Full Addition (A + B + CIN).
- When CTRL = 1: performs Full Subtraction (A - B - CIN).

### Sizing and Critical Path
- Uses transmission-gate XORs and multiplexed carry generation.
- Total Transistors: 24 MOSFETs.
