from controller import Supervisor
import random

supervisor = Supervisor()
timeStep = int(supervisor.getBasicTimeStep())

# Create a maze floor (on X–Y plane, Z = up)
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
    }}
    """
    root = supervisor.getRoot()
    children_field = root.getField("children")
    children_field.importMFNodeFromString(-1, floor_string)

# Create a wall at given position (Z is up)
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
    }}
    """
    root = supervisor.getRoot()
    children_field = root.getField("children")
    children_field.importMFNodeFromString(-1, wall_string)

# Generate a grid maze where cell_size and wall_width are accounted for exactly.
def generate_grid_maze(n=4, cell_size=1.0, wall_width=0.1, z_height=0.25, entrance=None, floor_height=0.1):
    """
    n: number of cells per side (n x n)
    cell_size: size of one cell (distance between inner faces of walls)
    wall_width: thickness of wall strips
    entrance: tuple (side, index) to leave an opening. side in {'N','S','E','W'},
              index is cell index along that side (0..n-1)
    """
    total = n * cell_size + (n + 1) * wall_width

    create_floor(width=total, length=total, height=floor_height)

    # Vertical wall segments (n+1 columns, each with n segments of height=cell_size)
    for col in range(n + 1):
        for row in range(n):
            # optional entrance on left/right outer vertical walls
            if entrance:
                side, idx = entrance
                if side == 'W' and col == 0 and row == idx:
                    continue
                if side == 'E' and col == n and row == idx:
                    continue
            x = col * (cell_size + wall_width)
            y = wall_width + row * (cell_size + wall_width)
            create_wall(x, y, 0, length=wall_width, width=cell_size, height=z_height)

    # Horizontal wall segments (n+1 rows, each with n segments of length=cell_size)
    for row in range(n + 1):
        for col in range(n):
            # optional entrance on top/bottom outer horizontal walls
            if entrance:
                side, idx = entrance
                if side == 'S' and row == 0 and col == idx:
                    continue
                if side == 'N' and row == n and col == idx:
                    continue
            x = wall_width + col * (cell_size + wall_width)
            y = row * (cell_size + wall_width)
            create_wall(x, y, 0, length=cell_size, width=wall_width, height=z_height)
    # add square corner blocks at every grid intersection so walls meet cleanly
    for i in range(n + 1):
        for j in range(n + 1):
            x = i * (cell_size + wall_width)
            y = j * (cell_size + wall_width)
            create_wall(x, y, 0, length=wall_width, width=wall_width, height=z_height)

# Example wrapper that creates a 4x4 maze with cell_size=1 and wall_width=0.1
# and an exit on the north side at cell index 1 (change entrance as needed).
def generate_simple_maze():
    generate_grid_maze(n=4, cell_size=1.0, wall_width=0.1, z_height=0.25, entrance=('N', 1), floor_height=0.1)

print("Generating maze on X–Y plane...")
generate_simple_maze()

while supervisor.step(timeStep) != -1:
    pass
