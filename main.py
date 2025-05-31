import heapq
from enum import Enum

class HexOrientation(Enum):
    POINTY_TOP = "pointy"
    FLAT_TOP = "flat"

class HexGrid:
    def __init__(self, grid, orientation = HexOrientation.POINTY_TOP):
        """
        Initialize hexagonal grid pathfinder.
        
        Args:
            grid: 2D list where 0 = walkable, 1 = obstacle
            orientation: Whether hexagons have pointy tops or flat tops
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0
        self.orientation = orientation
        
        # Hexagonal directions (6 neighbors)
        if orientation == HexOrientation.POINTY_TOP:
            # For pointy-top hexagons (odd-r horizontal layout)``
            self.directions = {
                0: (0, 1),   # East
                1: (-1, 1),  # Northeast (odd rows) / Southeast (even rows)
                2: (-1, 0),  # Northwest (odd rows) / Southwest (even rows)  
                3: (0, -1),  # West
                4: (1, 0),   # Southwest (odd rows) / Northwest (even rows)
                5: (1, 1)    # Southeast (odd rows) / Northeast (even rows)
            }
        else:
            # For flat-top hexagons
            self.directions = {
                0: (-1, 0),  # North
                1: (-1, 1),  # Northeast
                2: (1, 1),   # Southeast
                3: (1, 0),   # South
                4: (1, -1),  # Southwest
                5: (-1, -1)  # Northwest
            }
    
    def get_neighbors(self, row, col):
        """Get valid neighboring hexagon coordinates."""
        neighbors = []
        
        for direction, (dr, dc) in self.directions.items():
            if self.orientation == HexOrientation.POINTY_TOP:
                # Adjust for odd-r coordinate system
                if row % 2 == 1:  # Odd row
                    new_row = row + dr
                    new_col = col + dc
                else:  # Even row
                    new_row = row + dr
                    new_col = col + dc
                    if dr != 0:  # Diagonal moves need adjustment for even rows
                        new_col -= 1
            else:
                new_row = row + dr
                new_col = col + dc
                if col % 2 == 1 and dr != 0:  # Adjust for odd columns in flat-top
                    new_row += 1 if dr > 0 else -1
            
            if self.is_valid(new_row, new_col):
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def is_valid(self, row, col):
        """Check if coordinates are within bounds and walkable."""
        return (0 <= row < self.rows and 
                0 <= col < self.cols and 
                self.grid[row][col] == 0)
    
    def hex_distance(self, a, b):
        """Calculate hexagonal distance between two points."""
        r1, c1 = a
        r2, c2 = b
        
        if self.orientation == HexOrientation.POINTY_TOP:
            # Convert odd-r to cube coordinates for distance calculation
            def oddq_to_cube(row, col):
                x = col
                z = row - (col - (col & 1)) // 2
                y = -x - z
                return x, y, z
            
            x1, y1, z1 = oddq_to_cube(r1, c1)
            x2, y2, z2 = oddq_to_cube(r2, c2)
        else:
            # For flat-top, convert offset to cube coordinates
            def evenq_to_cube(row, col):
                x = col - (row - (row & 1)) // 2
                z = row
                y = -x - z
                return x, y, z
            
            x1, y1, z1 = evenq_to_cube(r1, c1)
            x2, y2, z2 = evenq_to_cube(r2, c2)
        
        return (abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)) / 2
    
    def get_direction(self, from_pos, to_pos):
        """Get direction index from one hex to adjacent hex."""
        dr = to_pos[0] - from_pos[0]
        dc = to_pos[1] - from_pos[1]
        
        for direction, (expected_dr, expected_dc) in self.directions.items():
            if self.orientation == HexOrientation.POINTY_TOP:
                if from_pos[0] % 2 == 0 and expected_dr != 0:  # Even row adjustment
                    expected_dc -= 1
                
                if dr == expected_dr and dc == expected_dc:
                    return direction
            else:
                if from_pos[1] % 2 == 1 and expected_dr != 0:  # Odd column adjustment
                    expected_dr += 1 if expected_dr > 0 else -1
                
                if dr == expected_dr and dc == expected_dc:
                    return direction
        
        return None
    
    def jump_in_direction(self, start, direction, goal, max_distance= 100):
        """
        Jump in a specific direction until finding a jump point or obstacle.
        
        For hexagonal grids, jump points occur when:
        1. We reach the goal
        2. We find forced neighbors (due to obstacles)
        3. We reach maximum jump distance
        """
        current = start
        distance = 0
        
        while distance < max_distance:
            # Get next position in direction
            neighbors = self.get_neighbors(current[0], current[1])
            next_pos = None
            
            # Find neighbor in the specified direction
            for neighbor in neighbors:
                if self.get_direction(current, neighbor) == direction:
                    next_pos = neighbor
                    break
            
            if next_pos is None or not self.is_valid(next_pos[0], next_pos[1]):
                return None
            
            current = next_pos
            distance += 1
            
            # Check if we reached the goal
            if current == goal:
                return current
            
            # Check for forced neighbors (jump point detection)
            if self.has_forced_neighbors(current, direction):
                return current
            
            # For hexagonal grids, we can also stop at regular intervals
            # to balance between jump distance and pathfinding accuracy
            if distance >= 3:  # Shorter jumps for hex grids
                return current
        
        return current
    
    def has_forced_neighbors(self, pos, came_from_direction):
        """
        Check if position has forced neighbors due to obstacles.
        In hexagonal grids, this is when we have walkable neighbors
        that would be pruned if not for nearby obstacles.
        """
        neighbors = self.get_neighbors(pos[0], pos[1])
        all_neighbors = self.get_all_hex_neighbors(pos[0], pos[1])
        
        # Count accessible neighbors vs total possible neighbors
        accessible_count = len(neighbors)
        total_possible = len([n for n in all_neighbors if 0 <= n[0] < self.rows and 0 <= n[1] < self.cols])
        
        # If we have obstacles blocking some directions, this could be a jump point
        return accessible_count < total_possible and accessible_count >= 3
    
    def get_all_hex_neighbors(self, row, col):
        """Get all 6 potential neighbor positions (including blocked ones)."""
        neighbors = []
        
        for direction, (dr, dc) in self.directions.items():
            if self.orientation == HexOrientation.POINTY_TOP:
                if row % 2 == 1:  # Odd row
                    new_row = row + dr
                    new_col = col + dc
                else:  # Even row
                    new_row = row + dr
                    new_col = col + dc
                    if dr != 0:
                        new_col -= 1
            else:
                new_row = row + dr
                new_col = col + dc
                if col % 2 == 1 and dr != 0:
                    new_row += 1 if dr > 0 else -1
            
            neighbors.append((new_row, new_col))
        
        return neighbors
    
    def find_path(self, start, goal):
        """
        Find path using A* with hexagonal jump point optimization.
        """
        if not self.is_valid(start[0], start[1]) or not self.is_valid(goal[0], goal[1]):
            return []
        
        if start == goal:
            return [start]
        
        # Priority queue: (f_score, g_score, position)
        open_set = [(0, 0, start)]
        heapq.heapify(open_set)
        
        closed_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.hex_distance(start, goal)}
        
        while open_set:
            current_f, current_g, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            # Explore neighbors with jump point optimization
            neighbors = self.get_neighbors(current[0], current[1])
            
            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue
                
                # Try to jump in the direction of this neighbor
                direction = self.get_direction(current, neighbor)
                if direction is not None:
                    jump_point = self.jump_in_direction(current, direction, goal)
                    if jump_point:
                        neighbor = jump_point
                
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + self.hex_distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.hex_distance(neighbor, goal)
                    
                    heapq.heappush(open_set, (f_score[neighbor], tentative_g, neighbor))
        
        return []


def print_hex_grid_with_path(grid, path, start, goal):
    """Print hexagonal grid with path visualization."""
    display_grid = [row[:] for row in grid]
    
    # Mark path
    for r, c in path:
        if (r, c) != start and (r, c) != goal:
            display_grid[r][c] = 2
    
    # Mark start and goal
    display_grid[start[0]][start[1]] = 3
    display_grid[goal[0]][goal[1]] = 4
    
    symbols = {0: '.', 1: '#', 2: 'P', 3: 'S', 4: 'G'}
    
    # Print with hexagonal offset for visualization
    for i, row in enumerate(display_grid):
        indent = "  " if i % 2 == 1 else ""  # Offset odd rows for hex appearance
        print(indent + ' '.join(symbols[cell] for cell in row))
    print()


if __name__ == "__main__":
    # Test hexagonal grid
    hex_grid = [
        [0, 0, 1, 0, 0, 0, 1, 0],
        [0, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0]
    ]

    # hex_grid = [
    #     [0, 0, 0, 0, 0, 0, 0, 0],
    #     [0, 0, 0, 1, 0, 0, 0, 0],
    #     [0, 0, 0, 1, 0, 0, 0, 0],
    #     [0, 0, 0, 1, 0, 0, 0, 0],
    #     [0, 0, 0, 1, 0, 0, 0, 0],
    #     [0, 0, 0, 1, 0, 0, 0, 0],
    #     [0, 0, 0, 1, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0, 0, 0, 0]
    # ]
    
    pathfinder = HexGrid(hex_grid, HexOrientation.POINTY_TOP)
    
    start = (0, 0)
    goal = (7, 7)
    
    print("Hexagonal Grid (. = walkable, # = obstacle):")
    print("(Odd rows are indented to show hexagonal structure)")
    print_hex_grid_with_path(hex_grid, [], start, goal)
    
    path = pathfinder.find_path(start, goal)
    
    if path:
        print(f"Path found with {len(path)} steps:")
        print("Path coordinates:", path)
        print(f"Total hexagonal distance: {pathfinder.hex_distance(start, goal):.1f}")
        print("\nHexagonal grid with path (* = path, S = start, G = goal):")
        print_hex_grid_with_path(hex_grid, path, start, goal)
    else:
        print("No path found!")
    
    # Demonstrate hexagonal distance calculation
    print(f"\nHexagonal distance examples:")
    test_points = [(0, 0), (2, 1), (4, 3), (7, 7)]
    for i, p1 in enumerate(test_points):
        for p2 in test_points[i+1:]:
            dist = pathfinder.hex_distance(p1, p2)
            print(f"Distance from {p1} to {p2}: {dist:.1f}")
