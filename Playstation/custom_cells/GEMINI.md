# Global Response Formatting Rule: Clean Plain-Text Markdown Only

Write all explanations, chat responses, documentation, walk-throughs, plans, reports, and markdown artifacts in clear, human-readable language using standard Markdown only.

## Strict Prohibition: No LaTeX or Raw Math Delimiters
- NEVER emit LaTeX or TeX formatting or math delimiters under any circumstances (unless the user explicitly requests raw LaTeX source code).
- Absolute ban on dollar signs for formatting: NEVER use single or double dollar signs (`$...$` or `$$...$$`) to enclose numbers, units, dimensions, multipliers, formulas, variables, or expressions. Dollar signs (`$`) are strictly reserved for shell variables (e.g. `$PATH`) or literal currency.
- Do NOT use LaTeX commands, environments, or syntax, including:
  `\(...\)`, `\[...\]`, `\text{...}`, `\frac{...}{...}`, `\times`, `\mu`, `\cdot`, `\approx`, `\le`, `\ge`, `\pm`, `\begin{...}`, `\end{...}`.

## Engineering, VLSI, and Circuit Metrics Formatting
When discussing semiconductor circuits, physical dimensions, timing metrics, and electrical quantities, always use clean plain text or inline code:
- **Transistor Geometry & Dimensions**:
  - Write `W = 2.0 um` (or `W = 2.0 µm`), `L = 0.18 um`, `W/L = 10/1`.
  - NEVER write `$W = 2.0\,\mu\text{m}$` or `$L = 0.18\,\mu\text{m}$`.
- **Multipliers & Buffer Drive Strengths**:
  - Write `2x`, `4x`, `1x`, `2X inverter`, `4X buffer`.
  - NEVER write `$2\times$` or `$4\times$`.
- **Timing Metrics & Delays**:
  - Write `t_pdLH = 170.8 ps`, `t_pdHL = 283.4 ps`, `t_rise = 163.2 ps`, `t_fall = 145.0 ps`.
  - Alternatively use inline code: `t_pdLH = 170.8 ps`.
  - NEVER write `$t_{pdLH} = 170.8\,\text{ps}$` or `$t_{rise} = ...$`.
- **Electrical Quantities, Loads & Units**:
  - Write `C_load = 20 fF`, `V_DD = 1.8 V`, `10 pF`, `1.5 mA`, `213.6 uW`, `10 kΩ` (or `10 kohm`).
  - NEVER write `$20\,\text{fF}$` or `($C_{load} = 20\,\text{fF}$)`.
- **Variable Names and Subscripts**:
  - Use underscores or plain subscripts: `V_DD`, `V_SS`, `V_th`, `I_ds`, `t_pd`, `C_in`.
  - NEVER use LaTeX subscript syntax inside dollar signs like `$V_{DD}$`.

## Mathematical and Logic Operations
- Express mathematics with readable plain text and standard Unicode symbols:
  - Multiplication: `x`, `*`, or `×` (e.g., `2 × 4` or `2 * 4` or `2x`)
  - Division & Fractions: `a / b`
  - Exponents / Powers: `x²`, `x³`, `10⁻¹²` or `10^-12`
  - Subscripts: `x₁`, `y₂` or underscore `x_1`, `y_2`
  - Logic & Boolean: `AND`, `OR`, `NOT`, `XOR`, `XNOR`, `&`, `|`, `^`, `~`
  - Comparison & Relations: `≤`, `≥`, `≠`, `≈`, `±`, `→`, `<=`, `>=`, `!=`, `~=`, `+/-`
- For complicated equations, explain them in plain words and write them on their own line using readable text.

## Scope of Application
This rule applies unconditionally to:
1. Direct chat responses and explanations.
2. Markdown artifacts and walkthroughs (e.g. `walkthrough.md`, `implementation_plan.md`).
3. Simulation and verification summaries and tables.
4. Code comments and documentation files.
