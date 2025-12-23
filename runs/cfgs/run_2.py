import math

from utils import Environment, Manipulator
env = Environment(width=13, height=13, grid_resolution=0.5)

env.add_rectangle_obstacle(6, 6, 1, 1)
env.add_rectangle_obstacle(8, 8, 1, 1)
env.add_rectangle_obstacle(2, 10, 1, 1)
env.add_circular_obstacle(8, 5, 0.8)
env.add_circular_obstacle(4, 9, 0.8)
env.add_circular_obstacle(3, 7, 0.8)

link_lengths = [3, 2, 3, 2.5, 1.2, 1.2]
link_widths = [1, 1, 1, 1, 1, 1]
manipulator = Manipulator(
    base_position=(2, 2),
    link_lengths=link_lengths,
    link_widths=link_widths,
    initial_angles=[0.506092, -1.165904, 0.1, 1.6, -0.298871, 1.106940]
)

# goal_angles = [0.012701, -0.482094, 1.263639, -0.69990, 1.530246, 1.253833]

goal_angles = [math.pi / 2 + math.pi / 10 , 0 - math.pi / 10,0,0,- math.pi / 2,0]

