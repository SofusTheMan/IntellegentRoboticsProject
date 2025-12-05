from dataclasses import dataclass

# Direction constants
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3


@dataclass
class Cell:
    # starts as if every side is a wall (True).
    # when it senses "no wall" it flips to False.
    wallN: bool = True
    wallE: bool = True
    wallS: bool = True
    wallW: bool = True
    visited: bool = False
    cost: float = float("inf")


class MazeMap:
    def __init__(self, rows: int, cols: int):
        self.rows = rows
        self.cols = cols
        self.grid = [[Cell() for _ in range(cols)] for _ in range(rows)]

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def get_cell(self, r: int, c: int) -> Cell:
        return self.grid[r][c]

    def set_wall(self, r: int, c: int, direction: int, exists: bool):
        """
        Set wall in this cell and matching wall in neighbour.
        exists=True  -> there is a wall
        exists=False -> opening (corridor / exit)
        """
        if not self.in_bounds(r, c):
            return

        cell = self.grid[r][c]

        if direction == NORTH:
            cell.wallN = exists
            nr, nc = r - 1, c
            if self.in_bounds(nr, nc):
                self.grid[nr][nc].wallS = exists

        elif direction == SOUTH:
            cell.wallS = exists
            nr, nc = r + 1, c
            if self.in_bounds(nr, nc):
                self.grid[nr][nc].wallN = exists

        elif direction == EAST:
            cell.wallE = exists
            nr, nc = r, c + 1
            if self.in_bounds(nr, nc):
                self.grid[nr][nc].wallW = exists

        elif direction == WEST:
            cell.wallW = exists
            nr, nc = r, c - 1
            if self.in_bounds(nr, nc):
                self.grid[nr][nc].wallE = exists

    def mark_visited(self, r: int, c: int):
        if self.in_bounds(r, c):
            self.grid[r][c].visited = True

    def is_visited(self, r: int, c: int) -> bool:
        if not self.in_bounds(r, c):
            return False
        return self.grid[r][c].visited

    def neighbours_open(self, r: int, c: int):
        """Returns list of (nr, nc, direction) where no wall between (r,c) & neighbour."""
        result = []
        if not self.in_bounds(r, c):
            return result

        cell = self.grid[r][c]

        # NORTH neighbour is r-1
        if not cell.wallN and self.in_bounds(r - 1, c):
            result.append((r - 1, c, NORTH))  #North

        # EAST neighbour is c+1
        if not cell.wallE and self.in_bounds(r, c + 1):
            result.append((r, c + 1, EAST))

        # SOUTH neighbour is r+1
        if not cell.wallS and self.in_bounds(r + 1, c):
            result.append((r + 1, c, SOUTH))

        # WEST neighbour is c-1
        if not cell.wallW and self.in_bounds(r, c - 1):
            result.append((r, c - 1, WEST))

        return result
