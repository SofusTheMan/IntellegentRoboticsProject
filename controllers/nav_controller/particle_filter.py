import random
from collections import Counter
from wall_perception import Walls # Used for type hinting and the sensor model

class ParticleFilter:
  

   
    MOTION_MODEL = {
        "FWD_SUCCESS": 0.8,
        "TURN_SUCCESS": 0.8
    }

    
    SENSOR_MODEL = {
        "MATCH": 0.7,
        "MISMATCH": 0.3
    }

    
    LOCALIZED_THRESHOLD = 0.9

   

    def __init__(self, maze_graph, num_particles):
      
        self.graph = maze_graph
        self.num_particles = num_particles
        
        self.all_cells = list(self.graph.cells.keys())
        self.all_orientations = ['N', 'E', 'S', 'W']
        
        self.particles = []
        self.weights = []
        
        self.initialize_randomly()

    def initialize_randomly(self):
       
        self.particles = []
        for _ in range(self.num_particles):
            pos = random.choice(self.all_cells)
            ori = random.choice(self.all_orientations)
            self.particles.append((pos, ori))
        
        
        self.weights = [1.0 / self.num_particles] * self.num_particles

    def predict(self, action):
       
        prob_fwd_success = self.MOTION_MODEL["FWD_SUCCESS"]
        prob_turn_success = self.MOTION_MODEL["TURN_SUCCESS"]
        
        new_particles = []
        
        for (pos, ori) in self.particles:
            r = random.random()
            
            if action == "FWD":
                if r < prob_fwd_success:
                    new_pos, new_ori = self._get_forward_state(pos, ori)
                else:
                    new_pos, new_ori = pos, ori
            
            elif action == "TURN_LEFT":
                if r < prob_turn_success:
                    new_pos, new_ori = pos, self._get_left_ori(ori)
                else:
                    new_pos, new_ori = pos, ori
            
            elif action == "TURN_RIGHT":
                if r < prob_turn_success:
                    new_pos, new_ori = pos, self._get_right_ori(ori)
                else:
                    new_pos, new_ori = pos, ori
            
            new_particles.append((new_pos, new_ori))
        
        self.particles = new_particles

    def update(self, measured_walls: Walls):
       
        
        for i in range(self.num_particles):
            
            particle_state = self.particles[i]
            true_walls = self._get_walls_for_particle(particle_state[0], particle_state[1])
            
            
            prob = self._calculate_sensor_prob(measured_walls, true_walls)
            
            
            self.weights[i] *= prob

        
        total_weight = sum(self.weights)
        
        if total_weight < 1e-9:
            
            print("WARNING: Particle weights collapsed. Re-initializing.")
            self.initialize_randomly()
        else:
            
            for i in range(self.num_particles):
                self.weights[i] /= total_weight

    def resample(self):
        
        new_particles = []
        n = self.num_particles
        
        
        r = random.uniform(0.0, 1.0 / n)
        c = self.weights[0]
        i = 0
        
        for m in range(n):
            u = r + m * (1.0 / n)
            while u > c:
                i += 1
                c += self.weights[i % n]
            
            new_particles.append(self.particles[i % n])
            
        self.particles = new_particles
        self.weights = [1.0 / n] * n

    def is_localized(self) -> bool:

        state_counts = Counter(self.particles)
        if not state_counts:
            return False

        most_common_state, count = state_counts.most_common(1)[0]
        return (count / self.num_particles) >= self.LOCALIZED_THRESHOLD

    def get_best_estimate(self):

        state_weights = {}
        for i in range(self.num_particles):
            p = self.particles[i]
            w = self.weights[i]
            state_weights[p] = state_weights.get(p, 0.0) + w

        if not state_weights:
            return None, None


        best_state = max(state_weights, key=state_weights.get)
        return best_state 


    def _get_forward_state(self, pos, ori):
        """Calculates the resulting state after a "FWD" action."""
        (x, y) = pos


        try:

            if ori == 'N' and not self.graph.cells[pos].get('N', True):
                return (x, y + 1), ori
            elif ori == 'S' and not self.graph.cells[pos].get('S', True):
                return (x, y - 1), ori
            elif ori == 'E' and not self.graph.cells[pos].get('E', True):
                return (x + 1, y), ori
            elif ori == 'W' and not self.graph.cells[pos].get('W', True):
                return (x - 1, y), ori
            else:
                return pos, ori

        except KeyError:

            return pos, ori


    def _get_left_ori(self, ori):

        turn_map = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
        return turn_map[ori]

    def _get_right_ori(self, ori):

        turn_map = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}
        return turn_map[ori]


    def _get_walls_for_particle(self, pos, ori) -> Walls:


        graph_walls = {}

        try:

            cell_data = self.graph.cells[pos]


            graph_walls = {
                'N': cell_data.get('N', True),
                'E': cell_data.get('E', True),
                'S': cell_data.get('S', True),
                'W': cell_data.get('W', True)
            }
            
            
        except KeyError:

            return Walls(front=True, right=True, left=True, behind=True)

        if ori == 'N':
            return Walls(front=graph_walls['N'], right=graph_walls['E'], left=graph_walls['W'], behind=graph_walls['S'])
        elif ori == 'E':
            return Walls(front=graph_walls['E'], right=graph_walls['S'], left=graph_walls['N'], behind=graph_walls['W'])
        elif ori == 'S':
            return Walls(front=graph_walls['S'], right=graph_walls['W'], left=graph_walls['E'], behind=graph_walls['N'])
        elif ori == 'W':
            return Walls(front=graph_walls['W'], right=graph_walls['N'], left=graph_walls['S'], behind=graph_walls['E'])

        return Walls(front=True, right=True, left=True, behind=True)

    def _calculate_sensor_prob(self, measured: Walls, true: Walls) -> float:
       
        prob = 1.0
        p_match = self.SENSOR_MODEL["MATCH"]
        p_mismatch = self.SENSOR_MODEL["MISMATCH"]
        
        
        prob *= p_match if measured.front == true.front else p_mismatch
       
        prob *= p_match if measured.right == true.right else p_mismatch
        
        prob *= p_match if measured.left == true.left else p_mismatch
        
        prob *= p_match if measured.behind == true.behind else p_mismatch
        
        return prob