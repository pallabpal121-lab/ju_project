# Universal Newton Multivariable BCD Accelerator - Production UVM Verification Environment

Industry-grade SystemVerilog UVM (IEEE 1800.2) verification environment for the Universal Multivariable Newton Block Coordinate Descent (BCD) Accelerator with 32-bit AXI4-Lite Slave interface (`newton_bcd_axi_lite.sv`).

---

## 1. Directory Structure

```
newton_multivar_32bit/
├── Makefile                          # Synopsys VCS compilation, simulation, and regression targets
├── env_vcs.sh                        # Tool license and environment variable initialization
├── README.md                         # Quickstart guide and architecture documentation
├── VERIFICATION_SIGNOFF.md           # Sign-off criteria, coverage goals, and test matrix
├── filelist/
│   ├── rtl.f                         # RTL compilation filelist
│   └── tb.f                          # UVM testbench compilation filelist
├── scripts/
│   ├── run_vcs.sh                    # VCS execution runner with plusargs
│   ├── run_regression.sh             # Full regression suite runner
│   ├── run_verdi.sh                  # Synopsys Verdi debugger launcher
│   └── gen_report.py                 # Summary and coverage report generator
├── uvm_tb/
│   ├── if/
│   │   └── newton_axi_if.sv          # AXI4-Lite + IRQ interface with clocking blocks & SVA
│   ├── trans/
│   │   └── newton_axi_seq_item.sv    # Sequence item with constraints for AXI & optimization parameters
│   ├── agent/
│   │   ├── newton_agent_config.sv    # Agent configuration object (is_active, checks, coverage)
│   │   ├── newton_sequencer.sv       # UVM Sequencer with 9 phases
│   │   ├── newton_driver.sv          # AXI4-Lite master driver implementing 9 phases & bus handshakes
│   │   ├── newton_monitor.sv         # AXI4-Lite bus monitor tracking programming, controls, status & IRQ
│   │   └── newton_agent.sv           # UVM Agent with 9 phases assembling sequencer, driver, monitor
│   ├── scb/
│   │   ├── newton_multivar_ref_model.sv # Golden C/SV bit-accurate BCD reference solver
│   │   └── newton_scoreboard.sv      # Scoreboard with comparator, tolerance checking, and 9 phases
│   ├── cov/
│   │   └── newton_coverage.sv        # Functional coverage subscriber covering registers, vars, status, sweeps
│   ├── env/
│   │   └── newton_env.sv             # UVM Environment connecting agent, scoreboard, coverage with 9 phases
│   ├── seq/
│   │   ├── newton_base_seq.sv        # Base sequence with AXI-Lite read/write & high-level helper tasks
│   │   ├── newton_multivar_quadratic_seq.sv # N-variable quadratic bowl optimization sequence
│   │   ├── newton_multivar_rosenbrock_seq.sv# Coupled multi-variable optimization sequence
│   │   ├── newton_multivar_random_seq.sv    # Constrained-random sweeps, parameters, and variable counts
│   │   ├── newton_multivar_corner_seq.sv    # Boundary conditions (N=2, N=16, zero tol, max sweeps=1)
│   │   └── newton_axi_stress_seq.sv         # Back-to-back AXI transactions and register stress sequence
│   ├── tests/
│   │   ├── newton_base_test.sv       # Base test with 9 phases and timeout watchdog
│   │   ├── newton_multivar_quadratic_test.sv # Test for quadratic functions
│   │   ├── newton_multivar_rosenbrock_test.sv# Test for coupled non-linear functions
│   │   ├── newton_multivar_random_test.sv    # Random regression test
│   │   ├── newton_multivar_corner_test.sv    # Corner and boundary test
│   │   └── newton_axi_stress_test.sv         # AXI protocol stress test
│   ├── pkg/
│   │   ├── newton_axi_agent_pkg.sv   # Agent package
│   │   ├── newton_axi_seq_pkg.sv     # Sequences package
│   │   ├── newton_axi_env_pkg.sv     # Scoreboard, coverage, environment package
│   │   ├── newton_axi_test_pkg.sv    # Test suite package
│   │   └── newton_axi_tb_pkg.sv      # Unified umbrella package
│   └── top/
│       └── newton_axi_tb_top.sv      # Testbench top module with clock/reset, DUT instantiation, and FSDB dump
```

---

## 2. DUT Architecture & Memory Map

The accelerator is an AXI4-Lite 32-bit slave coprocessor executing 2nd-order Newton optimization via Block Coordinate Descent (BCD).

### Memory-Mapped Register Space (12-bit Address Offset)
- `0x000` (`REG_CTRL`): Bit 0 = Start pulse (write 1 to launch optimization).
- `0x004` (`REG_STATUS`): Bit 0 = Busy, Bit 1 = Done sticky, Bits 4:2 = Status code (`STATUS_CONVERGED`, `STATUS_MAX_ITERS`, `STATUS_SINGULAR`).
- `0x008` (`REG_NUM_VARS`): Active dimension N (Range: 2 to 16).
- `0x00C` (`REG_TOLERANCE`): Q16.16 convergence threshold (Default: `0x0000_0080` ≈ 0.00195).
- `0x010` (`REG_ALPHA`): Q16.16 step size / learning rate (Default: `0x0001_0000` = 1.0).
- `0x014` (`REG_LAMBDA`): Q16.16 Levenberg-Marquardt damping factor (Default: `0x0000_0400` ≈ 0.0156).
- `0x018` (`REG_MAX_SWEEPS`): Maximum coordinate descent sweeps (Default: 50).
- `0x01C` (`REG_SWEEP_COUNT`): Executed sweeps count before termination.
- `0x020` (`REG_F_OPTIMAL`): Final cost function value f(x*).
- `0x024` (`REG_MAX_DELTA`): Maximum parameter update in last sweep.
- `0x040` (`REG_PROG_ADDR`): Microcode instruction memory address (0 to 63).
- `0x044` (`REG_PROG_DATA`): 32-bit packed micro-instruction word.
- `0x100 - 0x13C` (`STATE_VEC_WINDOW`): 16 words for x0 through x15 (Write: initial guess x_init, Read: optimal output x*).

---

## 3. Mandatory UVM 9-Phase Implementation

Every component in the hierarchy (`newton_driver`, `newton_monitor`, `newton_sequencer`, `newton_agent`, `newton_scoreboard`, `newton_coverage`, `newton_env`, and all tests) explicitly implements all 9 standard UVM phases:

1. `build_phase`: Factory construction (`type_id::create`), configuration retrieval via `uvm_config_db`.
2. `connect_phase`: Analysis port bindings, sequencer-to-driver bindings, virtual interface assignments.
3. `end_of_elaboration_phase`: Topology printing (`uvm_top.print_topology()`), interface connectivity checks.
4. `start_of_simulation_phase`: Simulation banners and initial parameter logging.
5. `run_phase`: Clock/reset generation, stimulus driving, bus monitoring, objection handling.
6. `extract_phase`: Extracting coverage counters and transaction statistics.
7. `check_phase`: Validating zero scoreboard mismatches and clean queues.
8. `report_phase`: Printing final verification report table (Pass/Fail, error counts, sweep counts).
9. `final_phase`: Clean simulation teardown.

---

## 4. How to Run

### Setup Environment
```bash
source env_vcs.sh
```

### Compile Testbench and RTL
```bash
make compile
```

### Run Specific Test
```bash
make quad        # Multivariable Quadratic Test
make rosenbrock  # Coupled Multivariable Test
make corner      # Corner Cases & Boundary Test
make random      # Constrained-Random Test
make stress      # AXI4-Lite Bus Stress Test
```

### Run Full Regression Suite
```bash
make regression
```

### View Waveforms in Synopsys Verdi
```bash
make verdi
```
