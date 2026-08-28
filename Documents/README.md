# Physical AI & Optimization: Comprehensive Study Guide & Simulator

Welcome to the **Interactive Study Guide & Simulator** dashboard. This application is designed specifically for your personal learning to bridge the gap between Stanford CS229 mathematical optimization foundations (Newton's Method, Logistic Regression, Hessian matrices) and real-world hardware implementation (Systolic Arrays, FPGA factorizers, Analog In-Memory Computing, Neuromorphic Chips).

---

## 📂 File Architecture
All files are located in your workspace under `/home/pallab-pal/ju_project/Documents/`:
*   [`index.html`](file:///home/pallab-pal/ju_project/Documents/index.html): The core study guide containing deep theoretical text, mathematical equations, and custom SVG block diagrams.
*   [`styles.css`](file:///home/pallab-pal/ju_project/Documents/styles.css): Custom dark-mode glassmorphic theme styling, math blocks, sticky-sidebar navigation, and simulator panel styling.
*   [`presentation.js`](file:///home/pallab-pal/ju_project/Documents/presentation.js): Navigation engine (sidebar scroll-spying) and the dynamic mathematical optimization simulator.

---

## 🚀 How to Open the Guide

### Option 1: Direct File Launch
Double-click `index.html` in your file manager, or run the following command in your terminal:
```bash
xdg-open index.html
```

### Option 2: Local HTTP Server (Recommended)
Starting a local server ensures smooth rendering and handles canvas scaling accurately:
```bash
# Start Python's built-in server
python3 -m http.server 8000

# Then open in your web browser: http://localhost:8000
```

---

## 📈 Guide & Simulator Features

### 1. Sticky Navigation Sidebar
As you scroll down the page, the sidebar will highlight the section you are currently reading. You can also click any section in the sidebar to jump directly to it.

### 2. Theoretical Sections
*   **Section 01: Physical AI Overview**: Understanding edge computing constraints, deterministic latency, and hardware-software co-design.
*   **Section 02: Gradient Descent (1st-Order)**: Explains the math of Gradient Descent and why it oscillates when the condition number of the Hessian matrix is high.
*   **Section 03: Newton's Method (2nd-Order)**: Includes the Taylor series expansion, definition of the Hessian matrix, and application to Logistic Regression / IRLS.
*   **Section 04: The Memory Wall & TPU**: Explains the von Neumann memory bottleneck and visualizes Google TPU-style 2D Systolic Arrays using an interactive SVG.
*   **Section 05: Silicon QR & Cholesky Solvers**: Explains how FPGAs factor matrices to solve $H \Delta \theta = -g$ in hardware using Givens Rotators and CORDIC engines.
*   **Section 06: Analog IMC & Neuromorphic Computing**: Details memristor crossbars (Kirchhoff's current law for matrix multiplies) and event-driven spikes on Intel Loihi.

### 3. Interactive Playground (Section 07)
You can visually compare **Gradient Descent (Cyan)** and **Newton's Method (Purple)**:
*   **Quadratic Bowl**: Newton's method reaches the global optimum in **exactly one step** (with step scale 1.0) because the quadratic Taylor expansion is a perfect representation of the bowl. Gradient descent takes many steps and oscillates due to uneven scaling.
*   **Rosenbrock Valley**: Gradient descent gets trapped in the curved valley floor and oscillates, taking hundreds of iterations. Newton's method calculates the curvature to take direct curved steps along the ridge straight to the minimum `(1, 1)`.
*   **Custom Start Point**: Click anywhere on the color contour map to choose a starting coordinate dynamically.
