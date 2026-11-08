# Multi-Target Radar Tracking System

## Project Overview
A software-based radar tracking simulation that tracks multiple moving targets, estimates their positions using a Kalman Filter, and displays tracking information through a web dashboard.

## Technologies Used
- C
- HTML
- CSS
- JavaScript
- JSON
- Kalman Filter
- GCC Compiler

## Features
- Multiple target tracking
- Target position and velocity calculation
- Target movement simulation
- Target history tracking
- Future position prediction
- Kalman Filter-based state estimation
- Measurement noise simulation
- Tracking accuracy analysis
- Error comparison chart
- Target monitoring alerts
- Interactive radar dashboard

## Project Structure
- `src/` – C source files
- `include/` – C header files
- `dashboard/` – Web dashboard files
- `radar_tracker.exe` – Compiled application
- `targets.json` – Tracking data

## How to Run

### Compile the C program
```bash
gcc -std=c11 -Wall -Wextra -Iinclude src/main.c src/tracker.c -o radar_tracker.exe -lm
```

### Run the program
```bash
./radar_tracker.exe
```

### Open the dashboard
Open `dashboard/index.html` using the VS Code Live Server extension.

## Project Purpose
To demonstrate multi-target tracking, position estimation, noise reduction, and future position prediction through a software simulation.

## Author
Joshna Mani