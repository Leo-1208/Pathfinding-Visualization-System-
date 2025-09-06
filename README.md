# Pathfinding Algorithm Visualizer

This project demonstrates and compares multiple **pathfinding algorithms** on a grid-based map.  
It supports both weighted and unweighted grids and provides detailed statistics for each algorithm.

## ✨ Features
- Implements **BFS, DFS, Dijkstra, Greedy Best-First, A*, and Bellman-Ford**.
- Handles **walls**, **weighted cells (1–9)**, and normal cells.
- Outputs:
  - Whether a path is found
  - Nodes explored
  - Execution time (in microseconds)
  - Path cost
- Saves marked grids (`output.txt`) showing explored nodes (`+`) and final path (`*`).

## 📂 Input Format
Input is read from `input.txt`:
Example Input

`input.txt`:
```text
5 7
S..#...
.#.##..
..#..G.
...##..
.......
```

## 📂 Output Format
Output:

```text
Console Table
Algorithm          Found   NodesExplored   Time(µs)     Cost
BFS                YES     17              120          11
DFS                YES     34              90           14
Dijkstra           YES     19              150          11
GreedyBestFirst    YES     12              80           12
A*                 YES     15              95           11
Bellman-Ford       YES     25              600          11
```
Best: A* (Cost=11)


output.txt (example for A*)
```
Original Grid
S..#...
.#.##..
..#..G.
...##..
.......

Marked Grid
*++#...
+#+##..
++#**G.
..+##*.
..++++*

* = final path
+ = explored nodes
# = wall
```

....
