"""
Adapter to make MazeGraph compatible with existing code that expects MazeMap interface.
This allows us to use the MazeGraph from maze_generator without changing all the existing code.
"""
import json
import os
from dataclasses import dataclass

# Direction constants (matching maze_map.py)
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3


@dataclass
class Cell:
    """Cell class compatible with MazeMap interface."""
    wallN: bool = True
    wallE: bool = True
    wallS: bool = True
    wallW: bool = True
    visited: bool = False
    cost: float = float("inf")


class MazeGraphAdapter:
    """
    Adapter that wraps a MazeGraph (loaded from file) to provide
    a MazeMap-like interface for compatibility with existing code.
    """
    def __init__(self, maze_graph_data):
        """
        Initialize from serialized MazeGraph data.
        maze_graph_data: dict with 'rows', 'cols', 'side_length', 'wall_cell_ratio', 'cells'
        """
        self.rows = maze_graph_data['rows']
        self.cols = maze_graph_data['cols']
        self.side_length = maze_graph_data['side_length']
        self.wall_cell_ratio = maze_graph_data['wall_cell_ratio']
        
        # Reconstruct cells dict from serialized format
        self.cells = {}
        for key, walls in maze_graph_data['cells'].items():
            x, y = map(int, key.split('_'))
            self.cells[(x, y)] = walls
        
        # Create grid for compatibility with MazeMap interface
        # Note: MazeGraph uses (x, y) where x is row, y is col
        # MazeMap uses (r, c) where r is row, c is col
        # So we map: r = x, c = y
        self.grid = [[Cell() for _ in range(self.cols)] for _ in range(self.rows)]
        
        # Populate grid from MazeGraph cells
        for (x, y), walls in self.cells.items():
            # x is row, y is col in MazeGraph
            r, c = x, y
            if 0 <= r < self.rows and 0 <= c < self.cols:
                cell = self.grid[r][c]
                # Convert MazeGraph wall format to Cell format
                # MazeGraph: {"N": True/False, "S": True/False, "E": True/False, "W": True/False}
                cell.wallN = walls.get("N", True)
                cell.wallS = walls.get("S", True)
                cell.wallE = walls.get("E", True)
                cell.wallW = walls.get("W", True)
    
    def in_bounds(self, r: int, c: int) -> bool:
        """Check if (r, c) is within bounds."""
        return 0 <= r < self.rows and 0 <= c < self.cols
    
    def get_cell(self, r: int, c: int) -> Cell:
        """Get cell at (r, c)."""
        return self.grid[r][c]
    
    def mark_visited(self, r: int, c: int):
        """Mark cell as visited."""
        if self.in_bounds(r, c):
            self.grid[r][c].visited = True
    
    def is_visited(self, r: int, c: int) -> bool:
        """Check if cell is visited."""
        if not self.in_bounds(r, c):
            return False
        return self.grid[r][c].visited
    
    def neighbours_open(self, r: int, c: int):
        """
        Returns list of (nr, nc, direction) where no wall between (r,c) & neighbour.
        Compatible with MazeMap.neighbours_open interface.
        """
        result = []
        if not self.in_bounds(r, c):
            return result
        
        cell = self.grid[r][c]
        
        # NORTH neighbour is r-1
        if not cell.wallN and self.in_bounds(r - 1, c):
            result.append((r - 1, c, NORTH))
        
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


def load_maze_graph_from_file(filepath=None):
    """
    Load MazeGraph from JSON file.
    If filepath is None, tries to find maze_graph.json in the nav_controller directory.
    """
    if filepath is None:
        # Try to find the file relative to this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(script_dir, 'maze_graph.json')
    
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"MazeGraph file not found: {filepath}. Make sure maze_generator has run first.")
    
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    return MazeGraphAdapter(data)

