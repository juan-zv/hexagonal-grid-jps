# Jump Point Search (JPS) for a 2D hexagonal grid

## Goal
The goal of this project is to implement Jump Point Search (JPS) for a 2D hexagonal grid. JPS is an optimization technique for pathfinding algorithms that reduces the number of nodes that need to be explored.

## Features
JPS is an optimization of the A* algorithm that skips over nodes that are not necessary to find the shortest path. It does this by identifying "jump points" in the grid, which are nodes that can be used to jump over other nodes. The algorithm prunes branches that are not necessary to find the shortest path, which reduces the number of nodes that need to be explored.

For a 2D hexagonal grid, the algorithm is adapted to account for the different movement directions and distances. A square grid has 8 possible movement directions, while a hexagonal grid has 6. The algorithm identifies jump points based on the movement directions and distances in the hexagonal grid.

## Instructions
1. Install uv
```python
pip install uv
```
2. Run the script using uv
```python
uv run main.py
```
🤓☝️ If you are interested in the original JPS algorithm for a square grid, you can run:
```python
uv run simple-jps.py
```