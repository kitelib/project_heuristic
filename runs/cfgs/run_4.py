import math

from utils import Environment, Manipulator
env = Environment(width=13, height=13, grid_resolution=0.5)

# Создаем манипулятор с 5 звеньями
link_lengths = [3, 2, 3, 1.5, 1.2, 1.2]
link_widths = [1, 1, 1, 1, 1, 1]
manipulator = Manipulator(
    base_position=(2, 2),
    link_lengths=link_lengths,
    link_widths=link_widths,
    initial_angles=[math.pi/ 2, - math.pi/6, - math.pi/6, - math.pi/16, math.pi/2, math.pi/4]
)

goal_angles = [math.pi/2, math.pi / 4, - math.pi / 2, 0, 0, -math.pi/2]
