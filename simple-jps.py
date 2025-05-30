import heapq
import math

class JumpPointSearch:
    def __init__(self, grid):
        """
        Initialize JPS with a grid where 0 = walkable, 1 = obstacle
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if grid else 0
        
        # 8-directional movement: N, NE, E, SE, S, SW, W, NW
        self.directions = [
            (-1, 0), (-1, 1), (0, 1), (1, 1),(1, 0), (1, -1), (0, -1), (-1, -1)
        ]
        
    def is_walkable(self, row, col):
        """Check if a cell is within bounds and walkable"""
        return (0 <= row < self.rows and 
                0 <= col < self.cols and 
                self.grid[row][col] == 0)
    
    def heuristic(self, a, b):
        """Octile distance heuristic for 8-directional movement"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
    
    def get_natural_neighbors(self, node, parent):
        """Get natural neighbors based on direction from parent"""
        if parent is None:
            # If no parent, return all walkable neighbors
            neighbors = []
            for dr, dc in self.directions:
                nr, nc = node[0] + dr, node[1] + dc
                if self.is_walkable(nr, nc):
                    neighbors.append((nr, nc))
            return neighbors
        
        # Calculate direction from parent to current node
        dr = node[0] - parent[0]
        dc = node[1] - parent[1]
        
        # Normalize direction
        if dr != 0:
            dr = dr // abs(dr)
        if dc != 0:
            dc = dc // abs(dc)
            
        neighbors = []
        
        if dr == 0:  # Horizontal movement
            # Continue in same direction
            if self.is_walkable(node[0], node[1] + dc):
                neighbors.append((node[0], node[1] + dc))
        elif dc == 0:  # Vertical movement
            # Continue in same direction
            if self.is_walkable(node[0] + dr, node[1]):
                neighbors.append((node[0] + dr, node[1]))
        else:  # Diagonal movement
            # Continue diagonally
            if self.is_walkable(node[0] + dr, node[1] + dc):
                neighbors.append((node[0] + dr, node[1] + dc))
            # Continue horizontally and vertically
            if self.is_walkable(node[0] + dr, node[1]):
                neighbors.append((node[0] + dr, node[1]))
            if self.is_walkable(node[0], node[1] + dc):
                neighbors.append((node[0], node[1] + dc))
                
        return neighbors
    
    def get_forced_neighbors(self, node, parent):
        """Get forced neighbors due to obstacles"""
        if parent is None:
            return []
            
        # Calculate direction from parent to current node
        dr = node[0] - parent[0]
        dc = node[1] - parent[1]
        
        # Normalize direction
        if dr != 0:
            dr = dr // abs(dr)
        if dc != 0:
            dc = dc // abs(dc)
            
        forced = []
        
        if dr == 0:  # Horizontal movement
            # Check for obstacles above/below and add forced neighbors
            for vertical_dir in [-1, 1]:
                if (not self.is_walkable(node[0] + vertical_dir, node[1] - dc) and
                    self.is_walkable(node[0] + vertical_dir, node[1])):
                    forced.append((node[0] + vertical_dir, node[1]))
        elif dc == 0:  # Vertical movement
            # Check for obstacles left/right and add forced neighbors
            for horizontal_dir in [-1, 1]:
                if (not self.is_walkable(node[0] - dr, node[1] + horizontal_dir) and
                    self.is_walkable(node[0], node[1] + horizontal_dir)):
                    forced.append((node[0], node[1] + horizontal_dir))
        else:  # Diagonal movement
            # Check for forced neighbors in diagonal movement
            if (not self.is_walkable(node[0] - dr, node[1]) and
                self.is_walkable(node[0] - dr, node[1] + dc)):
                forced.append((node[0] - dr, node[1] + dc))
            if (not self.is_walkable(node[0], node[1] - dc) and
                self.is_walkable(node[0] + dr, node[1] - dc)):
                forced.append((node[0] + dr, node[1] - dc))
                
        return forced
    
    def is_jump_point(self, node, parent, goal):
        """Check if a node is a jump point"""
        if node == goal:
            return True
            
        # Check for forced neighbors
        if self.get_forced_neighbors(node, parent):
            return True
            
        # For diagonal movement, check if we can reach jump points horizontally/vertically
        if parent is not None:
            dr = node[0] - parent[0]
            dc = node[1] - parent[1]
            
            if dr != 0 and dc != 0:  # Diagonal movement
                # Check horizontal direction
                if self.jump(node, (0, dc if dc != 0 else 1), goal) is not None:
                    return True
                # Check vertical direction
                if self.jump(node, (dr if dr != 0 else 1, 0), goal) is not None:
                    return True
                    
        return False
    
    def jump(self, node, direction, goal):
        """Jump in a direction until we hit a jump point, obstacle, or boundary"""
        dr, dc = direction
        current = node
        
        while True:
            next_node = (current[0] + dr, current[1] + dc)
            
            # Check bounds and walkability
            if not self.is_walkable(next_node[0], next_node[1]):
                return None
                
            current = next_node
            
            # Check if we've reached the goal
            if current == goal:
                return current
                
            # Check if current node is a jump point
            if self.is_jump_point(current, (current[0] - dr, current[1] - dc), goal):
                return current
    
    def get_successors(self, node, parent, goal):
        """Get jump point successors for a node"""
        successors = []
        
        # Get all valid neighbors (natural + forced)
        natural = self.get_natural_neighbors(node, parent)
        forced = self.get_forced_neighbors(node, parent)
        neighbors = natural + forced
        
        for neighbor in neighbors:
            # Calculate direction to neighbor
            dr = neighbor[0] - node[0]
            dc = neighbor[1] - node[1]
            
            # Jump in that direction
            jump_point = self.jump(node, (dr, dc), goal)
            if jump_point is not None:
                successors.append(jump_point)
                
        return successors
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
    
    def find_path(self, start, goal):
        """Find path using Jump Point Search"""
        if not self.is_walkable(start[0], start[1]) or not self.is_walkable(goal[0], goal[1]):
            return None
            
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        closed_set = set()
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
                
            closed_set.add(current)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            # Get parent for pruning
            parent = came_from.get(current, None)
            
            # Get jump point successors
            successors = self.get_successors(current, parent, goal)
            
            for successor in successors:
                if successor in closed_set:
                    continue
                    
                # Calculate distance between current and successor
                dx = abs(successor[0] - current[0])
                dy = abs(successor[1] - current[1])
                if dx == 0 or dy == 0:
                    distance = max(dx, dy)  # Straight line
                else:
                    distance = math.sqrt(dx*dx + dy*dy)  # Diagonal
                
                tentative_g = g_score[current] + distance
                
                if successor not in g_score or tentative_g < g_score[successor]:
                    came_from[successor] = current
                    g_score[successor] = tentative_g
                    f_score[successor] = tentative_g + self.heuristic(successor, goal)
                    heapq.heappush(open_set, (f_score[successor], successor))
        
        return None  # No path found

# Example usage and testing
if __name__ == "__main__":
    # Create a test grid (0 = walkable, 1 = obstacle)
    grid = [
        [0, 0, 0, 0, 1, 0, 1, 0],
        [0, 1, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 0, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 1, 0]
    ]
    
    jps = JumpPointSearch(grid)
    start = (0, 0)
    goal = (7, 7)
    
    path = jps.find_path(start, goal)
    
    if path:
        print(f"Path found with {len(path)} waypoints:")
        for i, (row, col) in enumerate(path):
            print(f"  {i+1}. ({row}, {col})")
        
        # Visualize the path
        print("\nGrid visualization (P = path, # = obstacle, . = empty):")
        for r in range(len(grid)):
            row_str = ""
            for c in range(len(grid[0])):
                if (r, c) in path:
                    if (r, c) == start:
                        row_str += "S "
                    elif (r, c) == goal:
                        row_str += "G "
                    else:
                        row_str += "P "
                elif grid[r][c] == 1:
                    row_str += "# "
                else:
                    row_str += ". "
            print(row_str)
    else:
        print("No path found!")