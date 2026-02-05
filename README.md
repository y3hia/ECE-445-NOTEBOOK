# Solar Scrubber

ECE 445 Senior Design — Spring 2026, Project #57.

An autonomous solar panel cleaning robot. Uses per-panel voltage sensing and a Perturb-and-Observe MPPT algorithm to detect when a specific panel is dirty (versus globally shaded or under cloud cover), then drives a 2-axis gantry to clean only that panel.

## Team
- Yehia Ahmed
- Sandra Georgy
- Jonathan Sengstock

**TA:** Chihun Song
**Professor:** Joohyung Kim

## Repository Layout
- `notebook/` — Daily lab notebook entries from project kickoff through the final demo.
- `firmware/` — Firmware iterations from initial breadboard tests through the final integrated controller.

## Subsystems
- **Sensing.** Three OPA376-buffered voltage dividers tap each panel in series, plus an INA241B current sense amplifier across the string shunt.
- **MPPT.** Discrete buck converter driven by a 50 kHz PWM from the ESP32-S3, with the Perturb-and-Observe algorithm running every 200 ms.
- **Control.** ESP32-S3-WROOM-1 running a finite state machine for autonomous operation, plus a serial-driven manual override mode.
- **Locomotion.** 2-axis gantry built on NEMA 23 steppers via DM542T drivers. Open-loop step counting against a hardcoded coordinate grid.
- **Cleaning.** Microfiber cloth dragged in a back-and-forth wiggle motion overlaid on each vertical sweep. Active spray and brush motor were dropped mid-semester due to power supply limitations.
