from controller import Supervisor
import random

supervisor = Supervisor()
timeStep = int(supervisor.getBasicTimeStep())

# Create a maze floor (on X–Y plane, Z = up)
def create_floor():
    floor_string = """
    Solid {
      translation 0 0 0
      children [
        Shape {
          appearance Appearance {
            material Material { diffuseColor 0.8 0.8 0.8 }
          }
          geometry Box {
            size 4 4 0.1
          }
        }
      ]
    }
    """
    root = supervisor.getRoot()
    children_field = root.getField("children")
    children_field.importMFNodeFromString(-1, floor_string)

# Create a wall at given position (Z is up)
def create_wall(x, y, z=0.25, length=1.0, width=0.1, height=0.5):
    wall_string = f"""
    Solid {{
      translation {x} {y} {z}
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

# Example maze: outer walls with one exit
def generate_simple_maze():
    z_height = 0.25  # wall center height above floor
    cell_size = 1.0

    # Outer walls around a 5x5 area
    for i in range(-2, 3):
        # Leave one exit on the top wall (y = 2)
        if i != 0:
            create_wall(i, 2, z_height, length=1.0, width=0.1)
        create_wall(i, -2, z_height, length=1.0, width=0.1)

    for j in range(-1, 2):
        create_wall(-2, j, z_height, length=0.1, width=1.0)
        create_wall(2, j, z_height, length=0.1, width=1.0)

print("Generating maze on X–Y plane...")
create_floor()
generate_simple_maze()

while supervisor.step(timeStep) != -1:
    pass
