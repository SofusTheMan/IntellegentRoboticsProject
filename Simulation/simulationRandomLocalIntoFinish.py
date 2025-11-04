import matplotlib.pyplot as plt
import numpy as np
import random
from collections import deque

class MazeGraph:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.edges = {}  # (x,y) -> set of neighbor (x,y)
        self.exit = None
        self.generate_maze()
        
    def generate_maze(self):
        """Generate maze using DFS to ensure connectivity"""
        visited = set()
        stack = [(0, 0)]
        
        while stack:
            current = stack[-1]
            visited.add(current)
            x, y = current
            
            # Find unvisited neighbors
            neighbors = []
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if (nx, ny) not in visited:
                        neighbors.append((nx, ny))
            
            if neighbors:
                # Choose random unvisited neighbor
                next_cell = random.choice(neighbors)
                
                # Add edge between current and next
                if current not in self.edges:
                    self.edges[current] = set()
                if next_cell not in self.edges:
                    self.edges[next_cell] = set()
                    
                self.edges[current].add(next_cell)
                self.edges[next_cell].add(current)
                
                stack.append(next_cell)
            else:
                stack.pop()
        
        # Pick a random edge cell as exit
        edge_cells = [(x, y) for x in range(self.width) for y in range(self.height)
                      if x == 0 or x == self.width-1 or y == 0 or y == self.height-1]
        self.exit = random.choice(edge_cells)
    
    def get_neighbors(self, pos):
        """Get connected neighbors of a position"""
        return self.edges.get(pos, set())
    
    def get_walls(self, pos, orientation):
        """Get walls around robot: [front, right, back, left]"""
        # Orientation: 0=North, 1=East, 2=South, 3=West
        x, y = pos
        dirs = [(0, -1), (1, 0), (0, 1), (-1, 0)]  # N, E, S, W
        
        walls = []
        for i in range(4):
            direction = (orientation + i) % 4
            dx, dy = dirs[direction]
            neighbor = (x + dx, y + dy)
            
            # Wall exists if neighbor not in edges or out of bounds
            has_wall = neighbor not in self.get_neighbors(pos)
            walls.append(has_wall)
        
        return walls

class RobotLocalizer:
    def __init__(self, maze_graph):
        self.graph = maze_graph
        self.true_pos = None
        self.true_orientation = None
        self.possible_states = []  # List of (pos, orientation)
        self.steps = 0
        
    def initialize(self):
        """Place robot randomly and create initial belief state"""
        # Random true position and orientation
        all_positions = list(self.graph.edges.keys())
        self.true_pos = random.choice(all_positions)
        self.true_orientation = random.randint(0, 3)
        
        # Initially, robot could be anywhere with any orientation
        self.possible_states = [(pos, ori) 
                                for pos in all_positions 
                                for ori in range(4)]
        
        # Filter based on initial sensor reading
        self.filter_by_sensors()
    
    def filter_by_sensors(self):
        """Filter possible states based on current sensor reading"""
        true_walls = self.graph.get_walls(self.true_pos, self.true_orientation)
        
        new_possible = []
        for pos, ori in self.possible_states:
            walls = self.graph.get_walls(pos, ori)
            if walls == true_walls:
                new_possible.append((pos, ori))
        
        self.possible_states = new_possible
    
    def find_path_to_exit(self):
        """Find shortest path from current position to exit using BFS"""
        from collections import deque
        
        queue = deque([(self.true_pos, [])])
        visited = {self.true_pos}
        
        while queue:
            pos, path = queue.popleft()
            
            if pos == self.graph.exit:
                return path
            
            for neighbor in self.graph.get_neighbors(pos):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        
        return []
    
    def get_action_to_move_to(self, target_pos):
        """Get action to move towards target position"""
        # Calculate direction to target
        dx = target_pos[0] - self.true_pos[0]
        dy = target_pos[1] - self.true_pos[1]
        
        # Determine required orientation
        if dx == 1 and dy == 0:
            required_ori = 1  # East
        elif dx == -1 and dy == 0:
            required_ori = 3  # West
        elif dx == 0 and dy == -1:
            required_ori = 0  # North
        elif dx == 0 and dy == 1:
            required_ori = 2  # South
        else:
            return None
        
        # Check if we need to turn
        if self.true_orientation == required_ori:
            return 'forward'
        elif (self.true_orientation + 1) % 4 == required_ori:
            return 'right'
        elif (self.true_orientation - 1) % 4 == required_ori:
            return 'left'
        else:
            # 180 degree turn, choose either direction
            return 'right'
    
    def make_move(self):
        """Make a move, navigating to exit if localized"""
        # If localized, navigate to exit
        if self.is_localized() and self.true_pos != self.graph.exit:
            path = self.find_path_to_exit()
            if path:
                action = self.get_action_to_move_to(path[0])
            else:
                action = 'forward'  # Fallback
        else:
            # Get valid moves (no wall in front)
            true_walls = self.graph.get_walls(self.true_pos, self.true_orientation)
            
            # Actions: prefer moving forward if possible
            if not true_walls[0]:  # No wall in front
                action = 'forward'
            else:
                # If can't move forward, randomly turn
                action = random.choice(['left', 'right'])
        
        if action == 'forward':
            dirs = [(0, -1), (1, 0), (0, 1), (-1, 0)]
            dx, dy = dirs[self.true_orientation]
            new_pos = (self.true_pos[0] + dx, self.true_pos[1] + dy)
            if new_pos in self.graph.get_neighbors(self.true_pos):
                self.true_pos = new_pos
        elif action == 'left':
            self.true_orientation = (self.true_orientation - 1) % 4
        elif action == 'right':
            self.true_orientation = (self.true_orientation + 1) % 4
        
        # Update possible states with same action (only if not localized)
        if not self.is_localized():
            new_possible = []
            for pos, ori in self.possible_states:
                if action == 'forward':
                    walls = self.graph.get_walls(pos, ori)
                    if not walls[0]:  # Can move forward
                        dirs = [(0, -1), (1, 0), (0, 1), (-1, 0)]
                        dx, dy = dirs[ori]
                        new_pos = (pos[0] + dx, pos[1] + dy)
                        if new_pos in self.graph.get_neighbors(pos):
                            new_possible.append((new_pos, ori))
                elif action == 'left':
                    new_possible.append((pos, (ori - 1) % 4))
                elif action == 'right':
                    new_possible.append((pos, (ori + 1) % 4))
            
            self.possible_states = new_possible
            self.filter_by_sensors()
        
        self.steps += 1
        
        return action
    
    def is_localized(self):
        """Check if robot knows its exact position and orientation"""
        return len(self.possible_states) == 1
    
    def visualize(self, ax):
        """Visualize the maze and robot states on given axis"""
        ax.clear()
        
        # Draw maze walls
        cell_size = 1.0
        for pos in self.graph.edges.keys():
            x, y = pos
            neighbors = self.graph.get_neighbors(pos)
            
            # Draw walls where there are no edges
            cx, cy = x * cell_size, y * cell_size
            
            # Check each direction
            if (x, y-1) not in neighbors:  # North wall
                ax.plot([cx, cx+cell_size], [cy, cy], 'k-', linewidth=2)
            if (x+1, y) not in neighbors:  # East wall
                ax.plot([cx+cell_size, cx+cell_size], [cy, cy+cell_size], 'k-', linewidth=2)
            if (x, y+1) not in neighbors:  # South wall
                ax.plot([cx, cx+cell_size], [cy+cell_size, cy+cell_size], 'k-', linewidth=2)
            if (x-1, y) not in neighbors:  # West wall
                ax.plot([cx, cx], [cy, cy+cell_size], 'k-', linewidth=2)
        
        # Draw outer boundary (except exit)
        for x in range(self.graph.width):
            for y in range(self.graph.height):
                cx, cy = x * cell_size, y * cell_size
                if (x, y) != self.graph.exit:
                    if y == 0:
                        ax.plot([cx, cx+cell_size], [cy, cy], 'k-', linewidth=3)
                    if y == self.graph.height - 1:
                        ax.plot([cx, cx+cell_size], [cy+cell_size, cy+cell_size], 'k-', linewidth=3)
                    if x == 0:
                        ax.plot([cx, cx], [cy, cy+cell_size], 'k-', linewidth=3)
                    if x == self.graph.width - 1:
                        ax.plot([cx+cell_size, cx+cell_size], [cy, cy+cell_size], 'k-', linewidth=3)
        
        # Mark exit in green
        ex, ey = self.graph.exit
        ax.plot(ex * cell_size + 0.5, ey * cell_size + 0.5, 'g*', markersize=20, label='Exit')
        
        # Draw possible positions (red) with orientation arrows
        dirs = [(0, -0.25), (0.25, 0), (0, 0.25), (-0.25, 0)]
        for pos, ori in self.possible_states:
            if (pos, ori) != (self.true_pos, self.true_orientation):
                x, y = pos
                ax.plot(x * cell_size + 0.5, y * cell_size + 0.5, 'ro', markersize=8, alpha=0.3)
                dx, dy = dirs[ori]
                ax.arrow(x * cell_size + 0.5, y * cell_size + 0.5, dx, dy, 
                        head_width=0.12, head_length=0.08, fc='red', ec='red', alpha=0.3)
        
        # Draw true position (blue)
        x, y = self.true_pos
        ax.plot(x * cell_size + 0.5, y * cell_size + 0.5, 'bo', markersize=12, label='True Position')
        
        # Draw orientation arrow
        dirs = [(0, -0.3), (0.3, 0), (0, 0.3), (-0.3, 0)]
        dx, dy = dirs[self.true_orientation]
        ax.arrow(x * cell_size + 0.5, y * cell_size + 0.5, dx, dy, 
                head_width=0.15, head_length=0.1, fc='blue', ec='blue')
        
        ax.set_xlim(-0.5, self.graph.width * cell_size + 0.5)
        ax.set_ylim(-0.5, self.graph.height * cell_size + 0.5)
        ax.set_aspect('equal')
        ax.invert_yaxis()
        ax.legend()
        ax.set_title(f'Step {self.steps} | Possible states: {len(self.possible_states)} | Localized: {self.is_localized()}')


# Main execution
print("Generating maze and initializing robot...")
maze = MazeGraph(8, 8)
robot = RobotLocalizer(maze)
robot.initialize()

import time

print(f"Robot placed at {robot.true_pos} facing {['North', 'East', 'South', 'West'][robot.true_orientation]}")
print(f"Exit at {maze.exit}")
print(f"Initial possible states: {len(robot.possible_states)}")
print("\nSimulation running (1 step per second)...")

fig, ax = plt.subplots(figsize=(12, 12))
plt.ion()
plt.show()

robot.visualize(ax)
plt.draw()
plt.pause(1)

while True:
    action = robot.make_move()
    print(f"Step {robot.steps}: Action={action}, Position={robot.true_pos}, Orientation={['N','E','S','W'][robot.true_orientation]}, Possible states={len(robot.possible_states)}")
    
    robot.visualize(ax)
    plt.draw()
    plt.pause(1)
    
    if robot.is_localized():
        print("\n✓ Robot is now localized!")
        if robot.true_pos == maze.exit:
            print("✓ Robot has exited the maze!")
            plt.ioff()
            plt.show()
            break