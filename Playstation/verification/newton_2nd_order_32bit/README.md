# Newton 2nd-Order Accelerator (32-bit Q16.16) - Synopsys UVM Verification

## Directory Organization

```
newton_2nd_order_32bit/
├── Makefile             # Main Synopsys VCS / Verdi verification Makefile
├── env_vcs.sh           # Source script for Synopsys licenses & tool paths
├── README.md            # Environment & verification documentation
├── work/                # VCS compilation database, simv executable, csrc, daidir
├── logs/                # Compilation log (compile.log) & simulation test logs
├── outputs/             # Waveform dumps (sim_waveform.vcd, fsdb)
├── reports/             # Functional coverage databases & summary reports
├── scripts/             # Shell execution scripts (run_vcs.sh, run_regression.sh, run_verdi.sh)
├── filelist/            # RTL (rtl.f) and UVM Testbench (tb.f) filelists
└── uvm_tb/              # Complete UVM verification testbench components
    ├── agent/           # Driver, Monitor, Sequencer, Agent
    ├── cov/             # Functional Coverage collector
    ├── env/             # UVM Environment
    ├── if/              # SystemVerilog Interface (newton_if)
    ├── pkg/             # Package definitions (newton_tb_pkg)
    ├── scb/             # Scoreboard & Golden Reference Model
    ├── seq/             # Directed & Random UVM Sequences
    ├── tests/           # UVM Tests (Quadratic, Cubic, Rational, Corner, Random)
    ├── top/             # Testbench Top-Level Module (newton_tb_top)
    └── trans/           # Sequence Item (newton_seq_item)
```

---

## Quick Start Guide

### 1. Source Environment
```bash
source env_vcs.sh
```

### 2. Available Make Targets
```bash
# View all targets
make help

# Default build & execute (compile + run default test)
make all   # or simply 'make'

# Compile RTL & UVM TB
make compile

# Run Complete 5-Test Regression Suite
make regression

# Run Individual Tests
make quad        # Quadratic optimization: f(x) = (x - 3)²
make cubic       # Cubic optimization: f(x) = x³ - 3x
make rational    # Rational/division: f(x) = x²/2 - x
make corner      # Boundary & corner cases
make random      # Constrained random sequence

# Run Custom Test with Options
make run TEST=newton_quadratic_test VERB=UVM_HIGH SEED=42

# Open Waveform in Verdi
make verdi

# Clean Generated Files
make clean
```
