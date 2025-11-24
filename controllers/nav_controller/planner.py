from collections import deque

class Planner:
    def __init__(self, rows=16, cols=16):
        self.rows = rows
        self.cols = cols


        self.dist = [
            [999 for _ in range(cols)]
            for _ in range(rows)
        ]


        self.goal = (rows // 2, cols // 2)

    def set_goal(self, row, col):
        self.goal = (row, col)

    def compute_distances(self, maze):


        for r in range(self.rows):
            for c in range(self.cols):
                self.dist[r][c] = 999


        q = deque()
        gr, gc = self.goal

        self.dist[gr][gc] = 0
        q.append((gr, gc))

        while q:
            r, c = q.popleft()
            d = self.dist[r][c]

            cell = maze.get_cell(r, c)

           
            if not cell["N"] and r + 1 < self.rows:
                if self.dist[r + 1][c] > d + 1:
                    self.dist[r + 1][c] = d + 1
                    q.append((r + 1, c))

           
            if not cell["E"] and c + 1 < self.cols:
                if self.dist[r][c + 1] > d + 1:
                    self.dist[r][c + 1] = d + 1
                    q.append((r, c + 1))

           
            if not cell["S"] and r - 1 >= 0:
                if self.dist[r - 1][c] > d + 1:
                    self.dist[r - 1][c] = d + 1
                    q.append((r - 1, c))

           
            if not cell["W"] and c - 1 >= 0:
                if self.dist[r][c - 1] > d + 1:
                    self.dist[r][c - 1] = d + 1
                    q.append((r, c - 1))

    def best_direction(self, row, col, orientation):

        current = self.dist[row][col]
        best = current
        best_dir = None

        neighbours = {
            "N": (row + 1, col),
            "E": (row, col + 1),
            "S": (row - 1, col),
            "W": (row, col - 1)
        }

        for d, (r, c) in neighbours.items():
            if 0 <= r < self.rows and 0 <= c < self.cols:
                if self.dist[r][c] < best:
                    best = self.dist[r][c]
                    best_dir = d

        return best_dir if best_dir is not None else None


    def unvisited_neighbours(self, row, col, maze):
        neighbours = []

        cell = maze.get_cell(row, col)


        if row + 1 < self.rows and not cell["N"]:
            if not maze.is_visited(row + 1, col):
                neighbours.append(("N", row + 1, col))


        if col + 1 < self.cols and not cell["E"]:
            if not maze.is_visited(row, col + 1):
                neighbours.append(("E", row, col + 1))

        if row - 1 >= 0 and not cell["S"]:
            if not maze.is_visited(row - 1, col):
                neighbours.append(("S", row - 1, col))


        if col - 1 >= 0 and not cell["W"]:
            if not maze.is_visited(row, col - 1):
                neighbours.append(("W", row, col - 1))

        return neighbours


    def pick_exploration_goal(self, row, col, maze):

        unvisited = self.unvisited_neighbours(row, col, maze)

        if len(unvisited) == 0:
            return None


        return unvisited[0]
