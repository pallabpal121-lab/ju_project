# Transistor Sizing & Full-Custom Layout Guide
## Logical Effort, Euler Paths, and Delay Optimization

---

## 1. Logical Effort and Sizing Methodology

In sub-micron CMOS custom cell design, delay of a logic gate is modeled by:
Delay (d) = g × h + p

Where:
- g = Logical Effort (intrinsic complexity of gate topology compared to an inverter)
- h = Electrical Effort (Load Capacitance C_out / Input Capacitance C_in)
- p = Parasitic Delay (internal junction and diffusion capacitances)

### Logical Effort Comparison for Custom Topologies

| Cell Topology | Typical Static CMOS | Custom Transmission-Gate / PTL | Advantage |
| :--- | :--- | :--- | :--- |
| **XOR2 (Sum/Difference)** | g = 4.0 (12T) | g = 1.5 (6T TG XOR) | **62% lower input cap**, 2.5x speed |
| **4:2 Compressor** | p = 8.0 (60T static) | p = 3.0 (28T TG-based) | **60% smaller delay**, minimal glitching |
| **2:1 Multiplexer** | g = 2.0 (12T static) | g = 1.0 (4T TG MUX) | **Zero static power**, 50% lower cap |
| **8T SRAM Cell** | N/A (32T DFF) | 8T (isolated read port) | **70% smaller layout area** |

---

## 2. PMOS to NMOS Width Ratio (Beta Ratio)

For symmetric rise and fall delays (t_r = t_f):
- Due to hole mobility (mu_p) being approximately 2x to 2.5x lower than electron mobility (mu_n) in silicon:
  W_p / W_n ≈ 2.0 to 2.2

In 130nm process nodes:
- Standard Inverter: W_n = 0.5 um, W_p = 1.0 um (L = 0.15 um)
- Transmission Gates: Sized symmetrically (W_n = 0.5 um, W_p = 0.8 um) to equalize ON-resistance across the full 0V to 1.8V input voltage swing.

---

## 3. Euler Path and Layout Continuity (Stick Diagramming)

To achieve maximum layout density with zero diffusion breaks:
1. Construct the dual graph of the NMOS pull-down and PMOS pull-up networks.
2. Find an unbroken **Euler Path** that traces every node in the graph exactly once.
3. Order the input polysilicon gate fingers according to the shared Euler path sequence.
4. Share source/drain diffusions between adjacent transistors to minimize parasitic junction capacitance (C_db and C_sb).
