from collections import deque
import heapq
# Note: NORTH, EAST, SOUTH, WEST are now imported from maze_graph_adapter
# The adapter provides the same interface as MazeMap, so this code works with both


class Planner:
    def __init__(self, rows=5, cols=5):
        self.rows = rows
        self.cols = cols
        self.goal_cells = [(rows // 2, cols // 2)]  

    def set_goal_cells(self, cells):
        """cells: list of (r,c) tuples."""
        self.goal_cells = cells

    
    def flood_fill(self, maze: MazeMap):
        # reset costs
        for r in range(maze.rows):
            for c in range(maze.cols):
                maze.grid[r][c].cost = float("inf")

        q = deque()
        for gr, gc in self.goal_cells:
            if maze.in_bounds(gr, gc):
                maze.grid[gr][gc].cost = 0
                q.append((gr, gc))

        while q:
            r, c = q.popleft()
            base_cost = maze.grid[r][c].cost
            for nr, nc, _d in maze.neighbours_open(r, c):
                if maze.grid[nr][nc].cost > base_cost + 1:
                    maze.grid[nr][nc].cost = base_cost + 1
                    q.append((nr, nc))

    def choose_next_cell(self, maze: MazeMap, r: int, c: int):
        """Pick neighbour with lowest flood-fill cost."""
        best = None
        best_cost = float("inf")
        for nr, nc, direction in maze.neighbours_open(r, c):
            cost = maze.grid[nr][nc].cost
            if cost < best_cost:
                best_cost = cost
                best = (nr, nc, direction)
        return best  # or None

    # Dijkstra shortest path 
    def dijkstra(self, maze: MazeMap, start, goal):
        # Grid-based Dijkstra from start to goal. Returns path as list of (r,c) 
        pq = []
        heapq.heappush(pq, (0, start))
        dist = {start: 0}
        prev = {}
        visited = set()

        while pq:
            cost, node = heapq.heappop(pq)
            if node in visited:
                continue
            visited.add(node)

            if node == goal:
                break

            r, c = node
            for nr, nc, _d in maze.neighbours_open(r, c):
                new_cost = cost + 1
                if (nr, nc) not in dist or new_cost < dist[(nr, nc)]:
                    dist[(nr, nc)] = new_cost
                    prev[(nr, nc)] = (r, c)
                    heapq.heappush(pq, (new_cost, (nr, nc)))

        if goal not in prev and goal != start:
            return []

        path = []
        cur = goal
        while cur != start:
            path.append(cur)
            cur = prev.get(cur)
            if cur is None:
                return []
        path.append(start)
        path.reverse()
        return path
