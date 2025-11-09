# A\* Pathfinding Algorithm in C

A simple implementation of the A\* pathfinding algorithm that finds the shortest path between two points on a grid.

## About

This program uses the A\* algorithm to find the optimal path from a starting point to an ending point while avoiding obstacles. It displays execution time, nodes explored, and visualizes the path on a grid.

## Features

- Finds shortest path using A\* algorithm
- User can input custom start and end points
- Shows execution time and statistics
- Visual grid display with path
- 8-directional movement (includes diagonals)

## Requirements

- C Compiler (GCC recommended)
- Math library

## How to Compile

```bash
gcc pathfinding.c -o pathfinding -lm
```

## How to Run

```bash
./pathfinding
```

## How to Use

1. Program displays the grid with obstacles
2. Enter starting point (x y coordinates)
3. Enter ending point (x y coordinates)
4. View the shortest path and statistics

## Grid Legend

- `.` = Free space
- `#` = Obstacle
- `S` = Start point
- `E` = End point
- `*` = Path

## Example

```
Enter starting point (x y): 0 0
Enter ending point (x y): 9 9

✓ Shortest path found!

--- Statistics ---
Execution time: 0.1234 ms
Nodes explored: 45
Path length: 15 steps
Path cost: 16.24
```

## Algorithm

A\* uses the formula: **f(n) = g(n) + h(n)**

- **g(n)** = Cost from start to current node
- **h(n)** = Estimated cost from current node to goal
- **f(n)** = Total estimated cost

## Customization

To change grid size, modify:

```c
#define GRID_SIZE 10
```

To add/remove obstacles, edit the grid array in main().
