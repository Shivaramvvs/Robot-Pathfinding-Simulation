# Robot-Pathfinding-Simulation
A Webots robotics simulation demonstrating autonomous maze exploration. Features a supervisor-robot architecture that allows seamless switching between Depth-First Search (DFS), Breadth-First Search (BFS), and A* pathfinding algorithms.

# How to Run & Switch Algorithms

The simulation supports three pathfinding algorithms: **DFS** (Default), **BFS**, and **ASTAR**. 

To switch between them, you just need to change the `controllerArgs` inside Webots. You do not need to edit the Python code.

**Step-by-Step Instructions:**
1. Open the simulation world (`.wbt` file) in Webots.
2. Ensure the simulation is paused and reset (⏪).
3. In the **Scene Tree** (left panel), double-click the **`DEF MAINSUPERVISOR Robot`** node to expand it.
4. Scroll down to the **`controllerArgs`** field and type your chosen algorithm: `"DFS"`, `"BFS"`, or `"ASTAR"`.
5. Next, expand the **`DEF EPUCK1 E-puck`** node.
6. Scroll down to its **`controllerArgs`** field and type the exact same algorithm name.
7. Save the world (`Ctrl + S`) and press **Play** (►).

*Note: Once the robot successfully locates the target, the simulation will automatically pause and generate a 2D map of the explored area.*
