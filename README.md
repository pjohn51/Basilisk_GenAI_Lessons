# Basilisk Astrodynamics — Learning Course

A complete Python notebook course for the [Basilisk Astrodynamics Simulation Framework](https://avslab.github.io/basilisk/)
developed by the AVS Lab at CU Boulder.

## Course Notebooks

| # | Notebook | Topics |
|---|---|---|
| 00 | `00_introduction.ipynb` | What is Basilisk, architecture, installation |
| 01 | `01_core_concepts.ipynb` | Simulation, Processes, Tasks, Modules, Messages |
| 02 | `02_basic_orbit.ipynb` | Two-body orbit, COEs, energy conservation |
| 03 | `03_spacecraft_dynamics.ipynb` | 6-DOF dynamics, RW state effectors |
| 04 | `04_attitude_dynamics.ipynb` | MRP, DCM, kinematics, RigidBodyKinematics |
| 05 | `05_fsw_attitude_control.ipynb` | Full FSW pipeline, MRP_Feedback control |
| 06 | `06_sensors_actuators.ipynb` | IMU, CSS, thrusters, noise models |
| 07 | `07_monte_carlo.ipynb` | MC analysis, statistical bounds, CDFs |
| 08 | `08_custom_modules.ipynb` | Python and C++ custom module development |
| 09 | `09_capstone.ipynb` | Full 6-DOF multi-phase mission simulation |

## Prerequisites

```bash
pip install bsk numpy matplotlib scipy jupyter
```

Or build from source:
```bash
git clone https://github.com/AVSLab/basilisk.git
cd basilisk && python conanfile.py && python -m build
```

## Quick Start

```bash
cd basilisk_learning
jupyter notebook 00_introduction.ipynb
```

## What You'll Build in the Capstone

A 10 kg spacecraft in a 500 km sun-synchronous orbit performing a 3-phase mission:
1. **Attitude acquisition** — stabilize from initial tumble
2. **Nadir pointing** — 30-minute science mode holding Earth-pointing
3. **Sun slew** — 90° reorientation maneuver

## References

- [Official Documentation](https://avslab.github.io/basilisk/)
- [GitHub Repository](https://github.com/AVSLab/basilisk)
- [Basilisk Paper (Kenneally 2020)](https://arc.aiaa.org/doi/10.2514/1.I010762)
