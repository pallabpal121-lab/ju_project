# 03_Debug_Waveforms_Verdi: Waveform Debug & Coverage Analysis

This folder contains the complete Synopsys Verdi debug platform manuals, FSDB waveform manipulation utilities, coverage analysis, and protocol verification guides.

---

## 1. Core Verdi & Verification Debug (Sequential Reading Path)

| Seq | Document Name | Description | Key Focus Topics |
| :--- | :--- | :--- | :--- |
| `01` | [01_VerdiTut_User_Guide_and_Tutorial.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/01_VerdiTut_User_Guide_and_Tutorial.pdf) | Verdi Main Tutorial & User Guide | nTrace schematic viewing, nWave waveform display, temporal flow |
| `02` | [02_CoverageTut_Coverage_User_Guide.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/02_CoverageTut_Coverage_User_Guide.pdf) | Verdi Coverage Tutorial | Code coverage (line, toggle, condition, FSM) and functional coverage |
| `03` | [03_UVMDebugUserGuide.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/03_UVMDebugUserGuide.pdf) | UVM Debug User Guide | UVM hierarchy browser, phase tracking, factory overrides, sequence tracing |
| `04` | [04_verdi_and_siloti_command_reference.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/04_verdi_and_siloti_command_reference.pdf) | Command Reference | CLI switches, Tcl commands, batch execution syntax |
| `05` | [05_LCAFeaturesGuide.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/05_LCAFeaturesGuide.pdf) | Limited Customer Availability Features | Advanced preview features, acceleration capabilities |
| `06` | [06_RDA_UserGuide_Regression_Debug_Automation.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/06_RDA_UserGuide_Regression_Debug_Automation.pdf) | Regression Debug Automation | Automated failure triage, error clustering, log mining |
| `07` | [07_verdi_ddt_ug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/07_verdi_ddt_ug.pdf) | Dynamic Driver Tracing | Root-cause tracing of unknown (X) states and signal drivers |
| `08` | [08_idx_assertion_verification_ug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/08_idx_assertion_verification_ug.pdf) | Assertion Verification Guide | SVA assertion debugging, failure time tracking, evaluation trees |
| `09` | [09_idx_ip_debug_user_guide.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/09_idx_ip_debug_user_guide.pdf) | IP Debug User Guide | Third-party IP protected-block debug and integration checks |
| `10` | [10_idx_waveform_replay_ug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/10_idx_waveform_replay_ug.pdf) | Waveform Replay Guide | Replaying simulations from FSDB checkpoints without rerun |
| `11` | [11_q_ref_Quick_Reference.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/11_q_ref_Quick_Reference.pdf) | Verdi Quick Reference | Keyboard shortcuts, mouse gestures, quick debug tricks |
| `12` | [12_upfa_ug_Power_Aware_Debug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/12_upfa_ug_Power_Aware_Debug.pdf) | UPF Power-Aware Debug | Power domain visualization, isolation cells, level shifters |

---

## 2. Advanced Verdi Verification & Specialized Engines

| Seq | Document Name | Description | Key Focus Topics |
| :--- | :--- | :--- | :--- |
| `13` | [13_Verdi_SVTB_Interactive_Debug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/13_Verdi_SVTB_Interactive_Debug.pdf) | SystemVerilog Testbench Interactive Debug | Breakpoints, thread inspector, class browser |
| `14` | [14_Verdi_Transaction_and_Protocol_Debug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/14_Verdi_Transaction_and_Protocol_Debug.pdf) | Transaction & Protocol Debug | High-level transaction streams, packet decoding, bus protocols |
| `15` | [15_Verdi_Power_Aware_Debug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/15_Verdi_Power_Aware_Debug.pdf) | Power-Aware Debug System | Multi-voltage states, power state table (PST) analysis |
| `16` | [16_Verdi_Performance_Analyzer.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/16_Verdi_Performance_Analyzer.pdf) | Performance Analyzer | Simulation bottleneck profiling, memory usage, execution speed |
| `17` | [17_Verdi_AMS_Debug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/17_Verdi_AMS_Debug.pdf) | Analog Mixed-Signal Debug | Co-viewing analog SPICE and digital RTL waveforms |
| `18` | [18_Verdi_HWSW_Debug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/18_Verdi_HWSW_Debug.pdf) | Hardware/Software Co-Debug | C/assembly software source tracing synchronized to RTL cycles |
| `19` | [19_Verdi_SystemC_Debug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/19_Verdi_SystemC_Debug.pdf) | SystemC Debug | SystemC kernel tracing, SC_METHOD/SC_THREAD thread inspection |
| `20` | [20_VCApps_Protocol_Analyzer.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/20_VCApps_Protocol_Analyzer.pdf) | Protocol Analyzer VC Apps | Protocol-specific checkers (PCIe, AXI, USB, DDR) |
| `21` | [21_VCS_Xprop.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/21_VCS_Xprop.pdf) | X-Propagation Analysis | Real-hardware X-pessimism and X-optimism debugging |
| `22` | [22_VC_Formal_FCA.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/22_VC_Formal_FCA.pdf) | Formal Coverage Analyzer | Reachability and coverage unreachability analysis |
| `23` | [23_FaultAnalysis.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/23_FaultAnalysis.pdf) | Fault Analysis | Functional safety fault injection and detection metrics |
| `24` | [24_SilotiTut.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/24_SilotiTut.pdf) | Siloti Visibility Enhancement | Minimum signal dumping with full internal state reconstruction |
| `25` | [25_SilotiCorrelation.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/25_SilotiCorrelation.pdf) | Siloti Correlation Guide | Correlation between physical trace buffers and simulation |
| `26` | [26_nAnalyzer.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/26_nAnalyzer.pdf) | nAnalyzer Guide | Gate-level netlist visualization, fan-in/fan-out tracing |
| `27` | [27_nECO.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/27_nECO.pdf) | Engineering Change Order (ECO) | Schematic-based functional ECO editing and patch generation |
| `28` | [28_nECO_tcl.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/28_nECO_tcl.pdf) | nECO Tcl Reference | Tcl automation for gate-level netlist patches |
| `29` | [29_ReplaySimulation.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/29_ReplaySimulation.pdf) | Simulation Replay Automation | Re-executing specific simulation intervals for debug |
| `30` | [30_linking_dumping.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/30_linking_dumping.pdf) | Waveform Linking & Dumping | PLI/VPI linking, `dumpvars`, `$fsdbDumpvars` commands |
| `31` | [31_tcl_Command_Reference.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/31_tcl_Command_Reference.pdf) | Verdi Tcl Command Reference | Complete Tcl command set for automation |
| `32` | [32_verdi_npi_upf_model.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/32_verdi_npi_upf_model.pdf) | Verdi NPI UPF Model | Programming interface for power intent objects |
| `33` | [33_verdi_supp_ug.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/33_verdi_supp_ug.pdf) | Supplementary User Guide | Additional utility features and debug modes |
| `34` | [34_verdi_supp_pin.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/34_verdi_supp_pin.pdf) | Supplementary Pin Guide | Pin mapping and netlist extraction |
| `35` | [35_COI_Main_Features.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/35_COI_Main_Features.pdf) | Cone of Influence (COI) | Cone of influence extraction and logic path isolation |
| `36` | [36_ChipInt_Main_Features.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/36_ChipInt_Main_Features.pdf) | Chip Integration Features | Top-level interconnect connectivity checking |
| `37` | [37_Euclide_IDE_Integration.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/37_Euclide_IDE_Integration.pdf) | Euclide Integration | IDE compilation and language-server integration |
| `38` | [38_installation.pdf](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/38_installation.pdf) | Verdi Installation Guide | License daemon, platform requirements, environment variables |

---

## 3. Subdirectories

* **[`API_and_Programming/`](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/API_and_Programming)**: Python NPI scripting, C/C++ FSDB Reader (`FsdbReader.pdf`) and Writer (`FsdbWriter.pdf`) libraries.
* **[`Example_Designs_and_Specs/`](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi/Example_Designs_and_Specs)**: Tutorial design collateral including PCI specifications, SRAM databooks, MemSys architecture, and GUI keybindings.
