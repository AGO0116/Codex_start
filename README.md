# robot-nav-viewer

3D robot viewer and navigation sandbox for legged and wheeled robots.

This project currently focuses on:

- `Unitree Go2` as a quadruped platform
- `TurtleBot3 Burger` as a wheeled platform
- 3D map viewing and manual teleoperation
- path planning and autonomous path tracking
- a separate 3D map editor

## What It Does

- Renders a 3D map in the browser with obstacles, route markers, and robot models
- Switches between `Go2` and `TurtleBot3 Burger`
- Supports manual robot control and camera control
- Plans routes with obstacle avoidance
- Shows a path-planning debug panel with graph / nodes / obstacle margins
- Opens a separate `Map Editor` screen for obstacle placement and spawn editing
- Lets you move and resize obstacles directly in 3D

## Current Robot Behavior

### Unitree Go2

- 3D quadruped model with jointed leg animation
- manual locomotion with body-frame controls
- polyline-based autonomous tracking
- route entry from the nearest point on the planned path

### TurtleBot3 Burger

- STL-based wheeled robot model
- curved autonomous tracking
- wheel animation during motion

Note:

- TurtleBot3 visual alignment is still being refined. If the base, wheels, or sensor stack look slightly off, that is known work in progress.

## Pages

### Viewer

- File: `web/index.html`
- Purpose: robot operation, path planning, autonomous navigation, debug view

### Editor

- File: `web/editor.html`
- Purpose: separate 3D map editing screen

## Main Files

- `web/index.html`
  Viewer UI and HUD
- `web/editor.html`
  Dedicated map editor screen
- `web/viewer.js`
  Main Three.js app, robot models, controls, planning, editor interactions
- `web/assets/go2/`
  Go2 visual assets and attribution
- `web/assets/turtlebot3/`
  TurtleBot3 visual assets and attribution
- `config/go2_map.json`
  Base map config
- `scripts/create_go2_map.py`
  Legacy 2D SVG map script

## Run

Start a simple local server from the repository root:

```bash
python -m http.server 8000
```

Open:

```text
http://localhost:8000/web/
```

Open the editor directly:

```text
http://localhost:8000/web/editor.html
```

## Requirements

- Python 3
- A browser with WebGL support
  - Google Chrome
  - Microsoft Edge
  - Firefox
  - Safari
- Internet access for CDN-loaded `three` / `three/addons`

## Viewer Controls

### Robot

- `W` move forward
- `A` move left
- `S` move backward
- `D` move right
- `Left / Right` yaw turn
- `F` sprint
- `Shift` sneak
- `Space` jump

### Camera

- `Camera` button switches between follow and free view
- In free view:
  - `LMB` orbit
  - `RMB` pan
  - `Wheel` zoom

### UI

- `E` toggles the on-screen control guide

## Path Planning Flow

1. `Start` places the route start marker
2. `Goal` places the goal marker
3. `Plan` computes a route with obstacle avoidance
4. `Auto` starts autonomous tracking

Route display:

- `Go2` uses the polyline route
- `TurtleBot3` uses the smoothed curve route

## Editor Controls

- `LMB` select objects
- `LMB drag` move selected obstacle
- `LMB drag on empty space` orbit camera
- `RMB drag` pan camera
- `Wheel` zoom
- Colored resize handles on the selected obstacle:
  - cyan: width
  - orange: depth
  - pink: height

## Editor Tools

### Session

- `Guide`
- `Close`

### Box Editing

- `Select`
- `Add Obstacle`
- `Delete Obstacle`

### Robot / Spawn

- `Set Spawn`

The right-side panel edits:

- map width / height
- obstacle default size
- selected obstacle position / size / elevation
- robot spawn yaw

## Assets

### Unitree Go2

- Product page: https://www.unitree.com/go2/
- ROS description: https://github.com/unitreerobotics/unitree_ros
- Model repository: https://github.com/unitreerobotics/unitree_model
- Attribution file: `web/assets/go2/ATTRIBUTION.md`

### TurtleBot3 Burger

- Repository: https://github.com/ROBOTIS-GIT/turtlebot3
- Reference URDF: `turtlebot3_description/urdf/turtlebot3_burger.urdf`
- Attribution file: `web/assets/turtlebot3/ATTRIBUTION.md`

## Legacy Script

The old 2D SVG map generator is still included:

```bash
python scripts/create_go2_map.py
```

Outputs:

- `output/go2_map.svg`

Uses:

- `config/go2_map.json`

## Notes

- This is a browser-based visualization / demo environment, not a full physics simulator
- Path tracking and gait behavior are tuned visually and interactively
- The editor and viewer share map state through browser storage
- Some robot visuals, especially TurtleBot3 STL alignment, may still need refinement

## Future Work

- Better TurtleBot3 mesh alignment
- More complete resize gizmos and editor handles
- Import / export for map files
- Cleaner module split for viewer, planner, and editor logic
- Stronger ROS integration
