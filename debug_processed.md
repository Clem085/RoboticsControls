---
title: Hummingbird Lab 3 - PD Control (Simulation)
date: 2025-10-31
header-includes:
  - \usepackage{amsmath}
  - \usepackage{amssymb}
  - \usepackage{siunitx}
  - \usepackage{graphicx}
  - \graphicspath{{../figs/}{figs/}{_hummingbird_sim/figs/}}
resource-path:
  - .
  - _hummingbird_sim
  - figs
  - ../figs
---

## Overview

- This note summarizes the PD designs used in Lab 3 for the hummingbird simulation and records the specific formulas used to compute gains from time-domain specs. All math is LaTeX to render correctly when exported to PDF (tested with pandoc and typical VS Code PDF exporters that support MathJax/KaTeX).

## Models

- Longitudinal (about $\theta_e=0$):

  

- Lateral (successive loop closure, inner roll faster than outer yaw):

  

## Dirty Derivative (First-Order Filtered Difference)

For any signal $x$:



## Gain Mapping from Specs

With desired $\zeta$ and rise time $t_r$:



- Longitudinal (pitch):

  

- Inner roll:

  

- Outer yaw:

  

## Bandwidth Separation

Choose inner-loop roll at least one order faster than yaw:



## Control Laws (PD Only)

- Longitudinal PD (force):

  

- Lateral PD (successive loops):

  Outer yaw (commands roll reference):

  

  Inner roll (commands torque):

  

## Simulation Notes

- H.7: Step $\theta_d$ by $\pm5^\circ$ and verify well-damped tracking. The logged force $F$ should equal $F_e$ plus PD action.
- H.8: Step $\psi_d$ (e.g., $30^\circ$). The roll response should be visibly faster; the rise-time ratio should be near $M$.

## How to Run

From the repository root (recommended to ensure imports resolve):

```bash
python -m _hummingbird_sim.h7_hummingbirdSim
python -m _hummingbird_sim.h8_hummingbirdSim
```

Or from the directory itself (matches current import style):

```bash
cd _hummingbird_sim
python h7_hummingbirdSim.py
python h8_hummingbirdSim.py
```

Python version used during development: `3.13`. If matplotlib/pyqt are missing, install from the repo root:

```bash
python -m pip install --user -r requirements.txt
```

## Results and Figures (placeholders)
GUI Simulation Output from H.7 & H.8
Images are expected in `../figs/` relative to this Markdown file (as you currently have in the repo). The YAML header sets a LaTeX `\graphicspath` and a Pandoc `resource-path` so PDF export can find them even if you run Pandoc from the repo root.

### H.7 - Longitudinal PD (Pitch)
![H.7 pitch tracking (theta vs theta_d)](../figs/image-3.png){ width=70% }
![H.7 force command F](../figs/image-2.png){ width=70% }

### H.8 - Lateral PD (Yaw/Roll)
![H.8 yaw tracking (psi vs psi_d)](../figs/image-1.png){ width=70% }
![H.8 roll inner-loop (phi vs phi_d)](../figs/image.png){ width=70% }

Tip: In matplotlib windows you can use the save icon, or temporarily add `plt.savefig('figs/h7_theta.png', dpi=200, bbox_inches='tight')` near the end of the script.

## PDF Export

Pandoc (recommended). Run from the file’s folder so relative image paths like `../figs/...` resolve the same as VS Code preview:

```bash
cd _hummingbird_sim
pandoc Lab3_Review.md \
  --standalone \
  --from gfm+tex_math_dollars+attributes+link_attributes \
  --to pdf \
  --pdf-engine=lualatex \
  -o Lab3_Review.pdf
```

Alternative (run from repo root) — include an explicit resource path so Pandoc finds figures stored outside this folder:

```bash
pandoc _hummingbird_sim/Lab3_Review.md \
  --standalone \
  --from gfm+tex_math_dollars+attributes+link_attributes \
  --to pdf \
  --pdf-engine=lualatex \
  --resource-path=".;_hummingbird_sim;figs;_hummingbird_sim/figs" \
  -o _hummingbird_sim/Lab3_Review.pdf
```

VS Code: pick a PDF exporter with MathJax/KaTeX enabled. If formulas render as plain text or images are missing, switch to Pandoc or use the `cd _hummingbird_sim` approach above.

## Repo Map (for this lab)

- Controllers: `ctrlLonPD.py`, `ctrlPD.py`
- Sims/plots: `h7_hummingbirdSim.py`, `h8_hummingbirdSim.py`, `dataPlotter.py`
- Dynamics/params: `hummingbirdDynamics.py`, `hummingbirdParam.py`, `signalGenerator.py`
- This report: `Lab3_Review.md`, lessons: `Lab3_Learned.txt`

## Troubleshooting

- If you see OpenGL/pyqtgraph warnings during animation, they are typically harmless on headless or constrained GPUs.
- If `_hummingbird_sim` imports fail when using `-m`, run from inside the folder with `python h7_hummingbirdSim.py`.
- If plots don’t show in PDF, confirm you used Pandoc with `lualatex` and that image paths exist.
