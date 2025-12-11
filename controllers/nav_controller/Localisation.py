import random
import math


class ParticleFilter:
    def __init__(self, num_particles=200, maze_map=None, cell_size=0.09, 
                 maze_rows=5, maze_cols=5, maze_side_length=0.6):
        
        # Initialize particle filter for unknown starting position.
        # Particles are distributed across all possible positions in the maze.
        
        self.num_particles = num_particles
        self.maze_map = maze_map
        self.cell_size = cell_size
        self.maze_rows = maze_rows
        self.maze_cols = maze_cols
        self.maze_side_length = maze_side_length
        
        # Calculate actual cell size from maze dimensions
        # Assuming similar structure to maze_generator
        wall_cell_ratio = 0.1
        wall_width = wall_cell_ratio * maze_side_length / maze_rows
        self.actual_cell_size = (maze_side_length - (maze_rows + 1) * wall_width) / maze_rows
        
        # Each particle: [x, y, theta, weight]
        self.particles = []
        self._initialize_particles()
        
    def _initialize_particles(self):
        # Initialize particles uniformly across all cells with random orientations
        wall_cell_ratio = 0.1
        wall_width = wall_cell_ratio * self.maze_side_length / self.maze_rows
        
        for _ in range(self.num_particles):
            # Random cell position
            cell_r = random.randint(0, self.maze_rows - 1)
            cell_c = random.randint(0, self.maze_cols - 1)
            
            # Convert cell to world coordinates (center of cell)
            x = wall_width + cell_c * (self.actual_cell_size + wall_width) + self.actual_cell_size / 2.0
            y = wall_width + cell_r * (self.actual_cell_size + wall_width) + self.actual_cell_size / 2.0
            
            # Random orientation (0, 90, 180, 270 degrees)
            theta = random.choice([0.0, math.pi/2, math.pi, -math.pi/2])
            
            self.particles.append([x, y, theta, 1.0 / self.num_particles])

    def predict(self, forward_dist=0.0, turn_angle=0.0,
                sigma_fwd=0.01, sigma_turn=0.02):
        """Move particles according to motion command + noise."""
        for p in self.particles:
            df = random.gauss(forward_dist, sigma_fwd)
            da = random.gauss(turn_angle, sigma_turn)
            p[2] += da
            p[0] += df * math.cos(p[2])
            p[1] += df * math.sin(p[2])

    def _get_expected_walls(self, x, y, theta):
        
        # Get expected wall configuration at particle pose.
       #  Returns: (front, right, left, behind) as booleans
        
        if self.maze_map is None:
            return (False, False, False, False)
        
        # Convert world coordinates to cell coordinates
        wall_cell_ratio = 0.1
        wall_width = wall_cell_ratio * self.maze_side_length / self.maze_rows
        
        # Find which cell this position is in
        cell_c = int((x - wall_width) / (self.actual_cell_size + wall_width))
        cell_r = int((y - wall_width) / (self.actual_cell_size + wall_width))
        
        if not (0 <= cell_r < self.maze_rows and 0 <= cell_c < self.maze_cols):
            return (True, True, True, True)  # Out of bounds = walls everywhere
        
        cell = self.maze_map.get_cell(cell_r, cell_c)
        
        # Determine which walls are in front/right/left/behind based on orientation
        # Normalize theta to [0, 2*pi)
        theta_norm = theta % (2 * math.pi)
        
        # Map orientation to directions: 0=E, π/2=N, π=W, -π/2=S
        if abs(theta_norm) < 0.1 or abs(theta_norm - 2*math.pi) < 0.1:  # East
            front_wall = cell.wallE
            right_wall = cell.wallS
            left_wall = cell.wallN
            behind_wall = cell.wallW
        elif abs(theta_norm - math.pi/2) < 0.1:  # North
            front_wall = cell.wallN
            right_wall = cell.wallE
            left_wall = cell.wallW
            behind_wall = cell.wallS
        elif abs(theta_norm - math.pi) < 0.1:  # West
            front_wall = cell.wallW
            right_wall = cell.wallN
            left_wall = cell.wallS
            behind_wall = cell.wallE
        else:  # South
            front_wall = cell.wallS
            right_wall = cell.wallW
            left_wall = cell.wallE
            behind_wall = cell.wallN
            
        return (front_wall, right_wall, left_wall, behind_wall)

    def update(self, ir_readings, walls_detected):
        
       # Update weights using sensor data.
     # walls_detected: Walls object with front, right, left, behind booleans
        
        if self.maze_map is None:
            # If no map, use uniform weights
            w = 1.0 / self.num_particles
            for p in self.particles:
                p[3] = w
            return
        
        # Compute likelihood for each particle
        for p in self.particles:
            x, y, theta = p[0], p[1], p[2]
            expected_walls = self._get_expected_walls(x, y, theta)
            
            # Compare expected vs detected walls
            # Likelihood based on how well walls match
            match_score = 0.0
            if expected_walls[0] == walls_detected.front:
                match_score += 1.0
            if expected_walls[1] == walls_detected.right:
                match_score += 1.0
            if expected_walls[2] == walls_detected.left:
                match_score += 1.0
            if expected_walls[3] == walls_detected.behind:
                match_score += 1.0
            
            # Convert match score to likelihood (exponential model)
            # Perfect match (4/4) = high weight, no match = low weight
            likelihood = math.exp(match_score - 2.0)  # Normalize so 2 matches = ~1.0
            
            p[3] = likelihood
        
        # Normalize weights
        total_weight = sum(p[3] for p in self.particles)
        if total_weight > 0:
            for p in self.particles:
                p[3] /= total_weight
        else:
            # If all weights are zero, reset to uniform
            w = 1.0 / self.num_particles
            for p in self.particles:
                p[3] = w

    def resample(self):
        """Resample particles according to weights."""
        weights = [p[3] for p in self.particles]
        total = sum(weights)
        if total == 0:
            return
        weights = [w / total for w in weights]
        new_particles = []
        for _ in range(self.num_particles):
            chosen = random.choices(self.particles, weights)[0]
            new_particles.append(chosen.copy())
        self.particles = new_particles

    def estimate_pose(self):
        """Return mean (x,y,theta) of particles, weighted by particle weights."""
        if len(self.particles) == 0:
            return 0.0, 0.0, 0.0
        
        # Weighted mean for position
        total_weight = sum(p[3] for p in self.particles)
        if total_weight == 0:
            # Fallback to unweighted mean
            x = sum(p[0] for p in self.particles) / len(self.particles)
            y = sum(p[1] for p in self.particles) / len(self.particles)
        else:
            x = sum(p[0] * p[3] for p in self.particles) / total_weight
            y = sum(p[1] * p[3] for p in self.particles) / total_weight
        
        # For theta, use weighted circular mean
        sin_sum = sum(math.sin(p[2]) * p[3] for p in self.particles)
        cos_sum = sum(math.cos(p[2]) * p[3] for p in self.particles)
        
        if abs(sin_sum) < 1e-6 and abs(cos_sum) < 1e-6:
            # Fallback to unweighted circular mean
            sin_sum = sum(math.sin(p[2]) for p in self.particles)
            cos_sum = sum(math.cos(p[2]) for p in self.particles)
        
        theta = math.atan2(sin_sum, cos_sum)
        
        return x, y, theta
    
    def get_best_particle(self):
        """Return the particle with highest weight."""
        best = max(self.particles, key=lambda p: p[3])
        return best[0], best[1], best[2]
