# IF4021 Modeling and Simulation — Boids Flocking Simulation (NetLogo)

Implementation of **Craig Reynolds' Boids algorithm** (1987) for emergent bird movement formation in **NetLogo 7.0.4**, accompanied by quantitative experiments and a LaTeX report.

## Repository Structure

- `boids.nlogox` — Self-contained NetLogo 7.0.4 simulation model implementing Reynolds' three steering behaviors (Separation, Alignment, Cohesion) with vector kinematics, speed bounds, steering force limits, field-of-view perception cone, and live flocking metrics.
- `docs/` — Simulation screenshots, experiment references, LaTeX report, and compiled PDF.
  - `docs/report.tex` — Reference LaTeX report template.
- `experiments/` — Headless simulation runs and quantitative analysis outputs.

## Algorithm Reference

- Reynolds, C. W. (1987). *Flocks, Herds, and Schools: A Distributed Behavioral Model*. Computer Graphics, 21(4), 25–34.
- Reynolds, C. W. (1999). *Steering Behaviors For Autonomous Characters*. Game Developers Conference.
