# SCL 180nm PDK Integration & Verification Guide (ts18scl)

This guide documents the integration of the **Semi-Conductor Laboratory (SCL) 180nm CMOS PDK** (`ts18scl`) with the full-custom cell design and transistor verification flow in Cadence Virtuoso and Spectre.

---

## 1. Directory Structure of SCL PDK Kit

The SCL 180nm PDK kit is installed at:
`/home/user17/Synopsys/SCLPDK_V3.0_KIT/scl180`

### Key PDK Components for Custom IC Flow

| Component | Path | Description |
| :--- | :--- | :--- |
| **Virtuoso OA Library** | `pdk/cdns/sclpdk_v3/HOTCODE/amslibs/cds_oa/cdslibs/ts18scl` | OpenAccess PDK library for transistor schematic & layout |
| **Spectre / SPICE Models** | `pdk/cdns/sclpdk_v3/HOTCODE/models/ts18scl/v2.0/hspice/ts18sl_scl.lib` | BSIM3v3.24 transistor model library |
| **Reference OA Symbols** | `pdk/cdns/sclpdk_v3/HOTCODE/amslibs/cds_oa/cdslibs/cds_generic` | Basic Cadence symbol primitives |

---

## 2. Cadence Virtuoso Integration

### Library Definition (`config/cds.lib`)
The project [config/cds.lib](file:///home/user17/Desktop/ju_project/Playstation/custom_cells/config/cds.lib) includes:
```
DEFINE ts18scl /home/user17/Synopsys/SCLPDK_V3.0_KIT/scl180/pdk/cdns/sclpdk_v3/HOTCODE/amslibs/cds_oa/cdslibs/ts18scl
DEFINE cds_generic /home/user17/Synopsys/SCLPDK_V3.0_KIT/scl180/pdk/cdns/sclpdk_v3/HOTCODE/amslibs/cds_oa/cdslibs/cds_generic
DEFINE sheetBorder /home/user17/Synopsys/SCLPDK_V3.0_KIT/scl180/pdk/cdns/sclpdk_v3/HOTCODE/amslibs/cds_oa/cdslibs/sheetBorder
DEFINE custom_cells_oa /home/user17/Desktop/ju_project/Playstation/custom_cells/oa_libs/custom_cells_oa
```

### Launching Virtuoso with SCL Environment
```bash
make virtuoso
```
Or via terminal:
```bash
source config/env_virtuoso.sh
virtuoso &
```

---

## 3. Transistor SPICE Simulation with Cadence Spectre

In Cadence Spectre, SCL 180nm BSIM3 models are included using HSPICE syntax mode:

```scs
simulator lang=spice
.lib '/home/user17/Synopsys/SCLPDK_V3.0_KIT/scl180/pdk/cdns/sclpdk_v3/HOTCODE/models/ts18scl/v2.0/hspice/ts18sl_scl.lib' tt_18

* Core 1.8V NMOS and PMOS device definitions in SCL 180nm
* Instance format: M<name> D G S B <model_name> L=<len> W=<width>
M1 (drain gate source bulk) n18.1 L=0.18u W=0.5u
M2 (drain gate source bulk) p18.1 L=0.18u W=1.0u

simulator lang=spectre
```

### Available PVT Model Corners

| Corner Section | Description | Nominal Voltage | Temperature |
| :--- | :--- | :--- | :--- |
| `tt_18` | Typical NMOS, Typical PMOS | 1.8 V | 25 °C |
| `ss_18` | Slow NMOS, Slow PMOS | 1.62 V | 125 °C |
| `ff_18` | Fast NMOS, Fast PMOS | 1.98 V | -40 °C |
| `fs_18` | Fast NMOS, Slow PMOS | 1.8 V | 25 °C |
| `sf_18` | Slow NMOS, Fast PMOS | 1.8 V | 25 °C |

---

## 4. Porting Custom Cells from Sky130 to SCL180

When adapting cell netlists in `cells/` to target SCL 180nm:
1. **Device Geometry Scaling**:
   - Channel length: scale from `L=0.15u` (Sky130) to `L=0.18u` (SCL 180nm).
   - Inverter sizing: NMOS `W=0.5u`, PMOS `W=1.0u` to `1.2u` for symmetric rise/fall times (`mu_n / mu_p ~ 2.4`).
2. **Model Card References**:
   - Replace `sky130_fd_pr__nfet_01v8` with `n18.1`.
   - Replace `sky130_fd_pr__pfet_01v8` with `p18.1`.
3. **Verification**:
   - Run `make sim CELL=<cell_name>` to verify transient switching, dynamic power, and propagation delays with SCL transistor models.
