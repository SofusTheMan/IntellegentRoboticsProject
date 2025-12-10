from controller import Supervisor
import random
import math
import json
import os

supervisor = Supervisor()
timeStep = int(supervisor.getBasicTimeStep())

# ==================== CONFIGURATION ====================
NUM_TRIALS = 100
TRIAL_TIMEOUT = 9000.0
RESULTS_FILE = "maze_solver_results.json"
# =======================================================

trial_results = []
current_trial = 0

def create_floor(width=4, length=4, height=0.25):
    floor_string = f"""
    Solid {{
      translation {width/2} {length/2} {-height/2}
      children [
        Shape {{
          appearance Appearance {{
            material Material {{ diffuseColor 0.8 0.8 0.8 }}
          }}
          geometry Box {{
            size {length} {width} {height}
          }}
        }}
      ]
      boundingObject Box {{
        size {length} {width} {height}
      }}
    }}
    """
    root = supervisor.getRoot()
    children_field = root.getField("children")
    children_field.importMFNodeFromString(-1, floor_string)

def create_wall(x, y, z=0.25, length=1.0, width=0.1, height=0.2):
    wall_string = f"""
    Solid {{
      translation {x+length/2} {y + width/2} {z+height/2}
      children [
        Shape {{
          appearance Appearance {{
            material Material {{ diffuseColor 0.2 0.2 0.2 }}
          }}
          geometry Box {{
            size {length} {width} {height}
          }}
        }}
      ]
      boundingObject Box {{
        size {length} {width} {height}
      }}
    }}
    """
    root = supervisor.getRoot()
    children_field = root.getField("children")
    children_field.importMFNodeFromString(-1, wall_string)

class MazeGraph:
    def __init__(self, rows, cols=None, side_length=4, wall_cell_ratio=0.1):
        print("Initializing MazeGraph rows:", rows, "cols:", cols, "side_length:", side_length, "wall_cell_ratio:", wall_cell_ratio)
        self.rows = rows
        if cols is None:
            cols = rows
        self.cols = cols
        self.side_length = side_length
        self.wall_cell_ratio = wall_cell_ratio
        self.cells = {
            (x, y): {"N": True, "S": True, "E": True, "W": True}
            for x in range(rows) for y in range(cols)
        }
        self.epuck_start = None

    def in_bounds(self, x, y):
        return 0 <= x < self.rows and 0 <= y < self.cols

    def neighbors(self, x, y):
        deltas = [(0, 1, "N"), (0, -1, "S"), (1, 0, "E"), (-1, 0, "W")]
        result = []
        for dx, dy, d in deltas:
            nx, ny = x + dx, y + dy
            if self.in_bounds(nx, ny):
                result.append((nx, ny, d))
        return result

    def remove_wall_between(self, a, b):
        (x1, y1), (x2, y2) = a, b
        dx, dy = x2 - x1, y2 - y1
        if (dx, dy) == (0, 1):
            self.cells[(x1, y1)]["N"] = False
            self.cells[(x2, y2)]["S"] = False
        elif (dx, dy) == (0, -1):
            self.cells[(x1, y1)]["S"] = False
            self.cells[(x2, y2)]["N"] = False
        elif (dx, dy) == (1, 0):
            self.cells[(x1, y1)]["E"] = False
            self.cells[(x2, y2)]["W"] = False
        elif (dx, dy) == (-1, 0):
            self.cells[(x1, y1)]["W"] = False
            self.cells[(x2, y2)]["E"] = False

    def carve_exit(self):
        side = random.choice(["N", "S", "E", "W"])
        if side in ["N", "S"]:
            idx = random.randint(0, self.cols - 1)
        else:
            idx = random.randint(0, self.rows - 1)
        self.carve_entrance(side, idx)

    def carve_entrance(self, side, index):
        if side == "N":
            self.cells[(index, self.cols - 1)]["N"] = False
        elif side == "S":
            self.cells[(index, 0)]["S"] = False
        elif side == "E":
            self.cells[(self.rows - 1, index)]["E"] = False
        elif side == "W":
            self.cells[(0, index)]["W"] = False

    def wall_lists(self):
        segments = []
        for (x, y), walls in self.cells.items():
            for side, present in walls.items():
                if present and (side=="N" or side=="S" and y==0 or side=="E" and x==self.cols-1 or side=="W"):
                    segments.append((x, y, side))
        return segments

    def generate_maze(self, z_height=0.25, floor_height=0.1):
        n = self.rows
        create_floor(width=self.side_length, length=self.side_length, height=floor_height)

        for (x, y, side) in self.wall_lists():
            self.place_wall_from_cell_side(x, y, side, z_height)

        wall_width = self.wall_cell_ratio * self.side_length / n
        cell_size  = (self.side_length - (n + 1) * wall_width) / n

        for i in range(n + 1):
            for j in range(n + 1):
                wx = i * (cell_size + wall_width)
                wy = j * (cell_size + wall_width)
                create_wall(wx, wy, 0, length=wall_width, width=wall_width, height=z_height)

    def place_wall_from_cell_side(self, x, y, side, z_height):
        n = self.rows
        wall_width = self.wall_cell_ratio * self.side_length / n
        cell_size  = (self.side_length - (n + 1) * wall_width) / n

        base_x = x * (cell_size + wall_width)
        base_y = y * (cell_size + wall_width)

        if side == "N":
            wx = wall_width + base_x
            wy = base_y + cell_size + wall_width
            create_wall(wx, wy, 0, length=cell_size, width=wall_width, height=z_height)
        elif side == "S":
            wx = wall_width + base_x
            wy = base_y
            create_wall(wx, wy, 0, length=cell_size, width=wall_width, height=z_height)
        elif side == "E":
            wx = base_x + cell_size + wall_width
            wy = wall_width + base_y
            create_wall(wx, wy, 0, length=wall_width, width=cell_size, height=z_height)
        elif side == "W":
            wx = base_x
            wy = wall_width + base_y
            create_wall(wx, wy, 0, length=wall_width, width=cell_size, height=z_height)

    def spawn_epuck_in_maze(self):
        n = self.rows
        m = self.cols

        side_length = self.side_length
        wall_cell_ratio = self.wall_cell_ratio
        wall_width = wall_cell_ratio * side_length / n
        cell_size = (side_length - (n + 1) * wall_width) / n

        cx = random.randrange(n)
        cy = random.randrange(m)

        tx = wall_width + cx * (cell_size + wall_width) + cell_size / 2.0
        ty = wall_width + cy * (cell_size + wall_width) + cell_size / 2.0
        tz = 0.0

        orientation = random.choice(['E', 'N', 'W', 'S'])
        self.epuck_start = (cx, cy, {'E': 1, 'N': 0, 'W': 3, 'S': 2}[orientation])
        angle_map = {'E': 0.0, 'N': math.pi / 2.0, 'W': math.pi, 'S': -math.pi / 2.0}
        angle = angle_map[orientation]

        epuck_node = f'''
        E-puck {{
          translation {tx} {ty} {tz}
          rotation 0 0 1 {angle}
          name "e_puck_{cx}_{cy}"
          controller "nav_controllerRemake"
        }}
        '''

        root = supervisor.getRoot()
        children_field = root.getField("children")
        children_field.importMFNodeFromString(-1, epuck_node)
        return (cx, cy, orientation)
    
    def serialize_to_file(self, filepath):
        data = {
            'rows': self.rows,
            'cols': self.cols,
            'side_length': self.side_length,
            'wall_cell_ratio': self.wall_cell_ratio,
            'cells': {f"{x}_{y}": walls for (x, y), walls in self.cells.items()},
            'epuck_start': self.epuck_start
        }
        with open(filepath, 'w') as f:
            json.dump(data, f)

def maze_generator_DFS(maze_graph):
    start_cell = (0, 0)
    stack = [start_cell]
    visited = {start_cell}
    
    while stack:
        unvisited_neighbors = []
        x, y = stack[-1]
        for nx, ny, _ in maze_graph.neighbors(x, y):
            if (nx, ny) not in visited:
                unvisited_neighbors.append((nx, ny))

        if len(unvisited_neighbors) > 0:
            next_cell = random.choice(unvisited_neighbors)
            maze_graph.remove_wall_between((x, y), next_cell)
            visited.add(next_cell)
            stack.append(next_cell)
        else:
            stack.pop()

    maze_graph.carve_exit()
    return maze_graph

def maze_hex_size(n, hex_size=1.0, wall_cell_ratio=0.1):
    side_length = n * hex_size
    return MazeGraph(n, side_length=side_length, wall_cell_ratio=wall_cell_ratio)

def save_results():
    """Save trial results to JSON."""
    completed = [t for t in trial_results if t['completed']]
    
    stats = {
        'total_trials': len(trial_results),
        'completed': len(completed),
        'failed': len(trial_results) - len(completed),
        'success_rate': round(len(completed) / len(trial_results) * 100, 1) if trial_results else 0
    }
    
    if completed:
        steps = [t['steps'] for t in completed]
        times = [t['time'] for t in completed]
        stats['steps'] = {'min': min(steps), 'max': max(steps), 'avg': round(sum(steps)/len(steps), 1)}
        stats['time'] = {'min': round(min(times), 2), 'max': round(max(times), 2), 'avg': round(sum(times)/len(times), 2)}
    
    with open(RESULTS_FILE, 'w') as f:
        json.dump({'statistics': stats, 'trials': trial_results}, f, indent=2)
    
    print(f"\n{'='*60}")
    print(f"RESULTS ({len(trial_results)} trials)")
    print(f"{'='*60}")
    print(f"Success: {len(completed)}/{len(trial_results)} ({stats['success_rate']}%)")
    if completed:
        print(f"Steps: {stats['steps']['min']}-{stats['steps']['max']} (avg {stats['steps']['avg']})")
        print(f"Time: {stats['time']['min']}-{stats['time']['max']}s (avg {stats['time']['avg']}s)")

# Initial setup
print(f"Starting {NUM_TRIALS} trials...")
rows = 5
maze_graph = maze_hex_size(rows, hex_size=0.09, wall_cell_ratio=0.1)
maze_graph = maze_generator_DFS(maze_graph)
maze_graph.generate_maze(z_height=0.05, floor_height=0.05)
maze_graph.spawn_epuck_in_maze()

maze_graph_file = os.path.join(os.path.dirname(__file__), '..', 'nav_controllerRemake', 'maze_graph.json')
maze_graph.serialize_to_file(maze_graph_file)

trial_start_time = supervisor.getTime()

# Main loop
while supervisor.step(timeStep) != -1:
    elapsed = supervisor.getTime() - trial_start_time
    
    # Check for robot completion signal
    signal_file = os.path.join(os.path.dirname(__file__), '..', 'nav_controllerRemake', 'trial_complete.json')
    if os.path.exists(signal_file):
        try:
            with open(signal_file, 'r') as f:
                data = json.load(f)
            os.remove(signal_file)
            
            trial_results.append({
                'trial': current_trial + 1,
                'completed': data['completed'],
                'timeout': False,
                'time': round(data['time'], 2),
                'steps': data['steps']
            })
            
            print(f"[Trial {current_trial + 1}] Success={data['completed']}, Steps={data['steps']}, Time={data['time']:.2f}s")
            save_results()
            
            current_trial += 1
            if current_trial >= NUM_TRIALS:
                print("\n=== ALL TRIALS COMPLETED ===")
                break
            
            # Reset and start next trial
            print(f"Starting trial {current_trial + 1}/{NUM_TRIALS}...")
            supervisor.simulationReset()
            for _ in range(10):
                supervisor.step(timeStep)
            
            maze_graph = maze_hex_size(rows, hex_size=0.09, wall_cell_ratio=0.1)
            maze_graph = maze_generator_DFS(maze_graph)
            maze_graph.generate_maze(z_height=0.05, floor_height=0.05)
            maze_graph.spawn_epuck_in_maze()
            maze_graph.serialize_to_file(maze_graph_file)
            
            trial_start_time = supervisor.getTime()
        except:
            pass
    
    # Check timeout
    if elapsed > TRIAL_TIMEOUT:
        trial_results.append({
            'trial': current_trial + 1,
            'completed': False,
            'timeout': True,
            'time': TRIAL_TIMEOUT,
            'steps': -1
        })
        
        print(f"[Trial {current_trial + 1}] TIMEOUT")
        save_results()
        
        current_trial += 1
        if current_trial >= NUM_TRIALS:
            print("\n=== ALL TRIALS COMPLETED ===")
            break
        
        print(f"Starting trial {current_trial + 1}/{NUM_TRIALS}...")
        supervisor.simulationReset()
        for _ in range(10):
            supervisor.step(timeStep)
        
        maze_graph = maze_hex_size(rows, hex_size=0.09, wall_cell_ratio=0.1)
        maze_graph = maze_generator_DFS(maze_graph)
        maze_graph.generate_maze(z_height=0.05, floor_height=0.05)
        maze_graph.spawn_epuck_in_maze()
        maze_graph.serialize_to_file(maze_graph_file)
        
        trial_start_time = supervisor.getTime()