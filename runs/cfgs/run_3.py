import math

from utils import Environment, Manipulator
env = Environment(width=13, height=13, grid_resolution=0.5)

env.add_rectangle_obstacle(6, 6, 1, 5)

env.add_rectangle_obstacle(8, 0,2,2)


link_lengths = [3, 2, 3, 1.5]
link_widths = [1, 1, 1, 1]
manipulator = Manipulator(
    base_position=(2, 2),
    link_lengths=link_lengths,
    link_widths=link_widths,
    initial_angles=[0.000000, 0.785398, -0.523599, 0.000000]
)

goal_angles = [1.570796, 0.785398, -1.570796, 0.785398]
