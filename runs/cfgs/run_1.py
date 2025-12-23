import math

from utils import Environment, Manipulator
env = Environment(width=13, height=13, grid_resolution=0.5)

env.add_rectangle_obstacle(0, 9, 1, 1)
env.add_circular_obstacle(10, 10, 3)
env.add_circular_obstacle(10, 2, 1.5)

link_lengths = [3, 2, 3, 1.5, 1.2, 1.2]
link_widths = [1, 1, 1, 1, 1, 1]
manipulator = Manipulator(
    base_position=(2, 2),
    link_lengths=link_lengths,
    link_widths=link_widths,
    initial_angles=[0, math.pi / 4, - math.pi/6, 0, -math.pi / 4, - math.pi/4]
)

goal_angles = [math.pi / 2, math.pi / 4, - math.pi / 2, math.pi/4, math.pi/4, -math.pi/2]