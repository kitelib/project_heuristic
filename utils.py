from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np


factor_by_2pi: Callable[[float], float] = lambda angle: ((angle + np.pi) % (2 * np.pi)) - np.pi

@dataclass
class Joint:
    """Сустав манипулятора"""
    length: float  # Длина звена
    angle: float   # Угол относительно предыдущего звена in Rad
    width: float   # Ширина звена
    
    def __post_init__(self):
        # Нормализуем угол в диапазон [-\pi, \pi]
        self.angle = factor_by_2pi(self.angle)

@dataclass
class ManipulatorState:
    """Набор углов всех суставов"""
    angles: list[float]  # Углы каждого сустава
    position: tuple[float, float] = (0, 0)  # Базовая позиция манипулятора
    
    def __post_init__(self):
        # Нормализуем углы
        self.angles = [factor_by_2pi(angle) for angle in self.angles]
    
    def __len__(self):
        return len(self.angles)
    
    def copy(self):
        return ManipulatorState(angles=self.angles.copy(), position=self.position)

class Manipulator:
    def __init__(
        self, 
        base_position: tuple[float, float], 
        link_lengths: list[float], 
        link_widths: list[float], 
        initial_angles: Optional[list[float]] = None,
    ):
        self.base_position = np.array(base_position, dtype=float)
        self.link_lengths = link_lengths
        self.link_widths = link_widths
        self.num_joints = len(link_lengths)
        
        if initial_angles is None:
            initial_angles = [0] * self.num_joints
        elif len(initial_angles) != self.num_joints:
            raise ValueError(f"Колво углов: ({len(initial_angles)}) != колво звеньев: ({self.num_joints})")
        
        self.state = ManipulatorState(angles=initial_angles, position=base_position)
        
    def get_joint_positions(
        self, 
        state: Optional[ManipulatorState] = None,
    ) -> list[np.ndarray]:
        """
        Получение позиций всех суставов манипулятора
        """
        if state is None:
            state = self.state
        
        positions: list[np.ndarray] = [np.array(state.position)]
        current_angle = 0
        
        for i in range(self.num_joints):
            current_angle += state.angles[i]
            x = positions[-1][0] + self.link_lengths[i] * np.cos(current_angle)
            y = positions[-1][1] + self.link_lengths[i] * np.sin(current_angle)
            positions.append(np.array([x, y]))
        
        return positions
    
    def get_link_rectangles(
        self, 
        state: Optional[ManipulatorState] = None,
    ) -> list[tuple[np.ndarray, float, float]]:
        """
        Получение звеньев манипулятора
        """
        if state is None:
            state = self.state
        
        positions = self.get_joint_positions(state)
        rectangles = []
        
        for i in range(self.num_joints):
            start = positions[i]
            end = positions[i+1]
            
            center = (start + end) / 2
            
            length = np.linalg.norm(end - start)  # compute L2 norm
            width = self.link_widths[i]
            
            angle = np.arctan2(end[1] - start[1], end[0] - start[0])
            
            rectangles.append((center, length, width, angle))
        
        return rectangles
    
    def get_end_effector_position(
        self, 
        state: Optional[ManipulatorState] = None,
    ) -> np.ndarray:
        """
        Получение позиции концевика манипулятора
        """
        if state is None:
            state = self.state
        return self.get_joint_positions(state)[-1]
    
    def set_state(
        self, 
        state: ManipulatorState,
    ):
        self.state = state

class Environment:
    """
    Среда, в которой существует маниулятор
    """
    
    def __init__(
        self, 
        width: int, 
        height: int, 
        grid_resolution: float = 1,
    ):
        self.width = width
        self.height = height
        self.grid_resolution = grid_resolution
        self._grid_cache = {} 
        
        # Создаем сетку препятствий  с логикой: (False - свободно, True - занято)
        self.grid_width = int(width / grid_resolution)
        self.grid_height = int(height / grid_resolution)
        self.obstacle_grid: np.ndarray[int] = np.zeros((self.grid_height, self.grid_width), dtype=bool)
        
        self.obstacles: list[dict[str, Any]] = []
        
    def add_obstacle_grid_cells(
        self, 
        cells: list[tuple[int, int]]
    ):
        """Добавление препятствий по клеткам сетки"""
        for cell in cells:
            x, y = cell
            if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                self.obstacle_grid[y, x] = True
    
    def add_rectangle_obstacle(
        self, 
        x: float,
        y: float, 
        width: float, 
        height: float,
    ):
        """Добавление прямоугольного препятствия"""
        # важно чтоб координаты были интами тк они отвечают индексам
        grid_x_start = int(x / self.grid_resolution)
        grid_y_start = int(y / self.grid_resolution)
        grid_x_end = int((x + width) / self.grid_resolution)
        grid_y_end = int((y + height) / self.grid_resolution)
        
        grid_x_start = max(0, grid_x_start)
        grid_y_start = max(0, grid_y_start)
        grid_x_end = min(self.grid_width, grid_x_end)
        grid_y_end = min(self.grid_height, grid_y_end)
        
        # Заполняем клетки
        for i in range(grid_x_start, grid_x_end):
            for j in range(grid_y_start, grid_y_end):
                self.obstacle_grid[j, i] = True
        
        self.obstacles.append({
            'type': 'rectangle',
            'x': x, 'y': y, 'width': width, 'height': height
        })
    
    def add_circular_obstacle(
        self,
        x: float, 
        y: float, 
        radius: float,
    ):
        """Добавление круглого препятствия"""
        # Преобразуем в клетки сетки
        for i in range(self.grid_width):
            for j in range(self.grid_height):
                cell_x = i * self.grid_resolution + self.grid_resolution / 2
                cell_y = j * self.grid_resolution + self.grid_resolution / 2
                
                # Проверяем, находится ли центр клетки внутри круга
                if (cell_x - x)**2 + (cell_y - y)**2 <= radius**2:
                    self.obstacle_grid[j, i] = True
        
        # Добавляем для отрисовки
        self.obstacles.append({
            'type': 'circle',
            'x': x, 'y': y, 'radius': radius
        })
    
    def is_in_collision_grid(self, point: np.ndarray) -> bool:
        """Проверка, находится ли точка в препятствии по сетке"""
        grid_x = int(point[0] / self.grid_resolution)
        grid_y = int(point[1] / self.grid_resolution)

        cahe_key = (grid_x, grid_y)
        
        if cahe_key in self._grid_cache:
            return self._grid_cache[cahe_key]
        
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            result = self.obstacle_grid[grid_y, grid_x]
        else:
            result = True
        
        self._grid_cache[cahe_key] = result
        return result
    
    def clear_cache(self):
        self._grid_cache.clear()
        
    
    def is_in_collision_rectangle(self, 
        rect_center: np.ndarray, 
        length: float, 
        width: float, 
        angle: float,
    ) -> bool:
        """
        Проверка столкновения прямоугольника с препятствиями
        """
        # Создаем повернутый прямоугольник и проверяем его углы
        half_length = length / 2
        half_width = width / 2
        
        # Углы прямоугольника до поворота
        corners_local = np.array([
            [-half_length, -half_width],
            [half_length, -half_width],
            [half_length, half_width],
            [-half_length, half_width]
        ])
        
        # Матрица поворота
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        rotation_matrix = np.array(
            [
                [cos_a, -sin_a], 
                [sin_a, cos_a]
            ]
        )
        
        corners_global = corners_local @ rotation_matrix.T + rect_center
        
        # Проверяем каждый угол и несколько точек вдоль сторон
        for corner in corners_global:
            if self.is_in_collision_grid(corner):
                return True
            
        for i in range(5):
            t = i / 4
            for side in [0, 1]:  # Две длинные стороны
                if side == 0:
                    point_local = np.array([-half_length + t * length, -half_width])
                else:
                    point_local = np.array([-half_length + t * length, half_width])
                
                point_global = point_local @ rotation_matrix.T + rect_center
                if self.is_in_collision_grid(point_global):
                    return True
        
        return False
    
    def check_manipulator_collision(
        self, 
        manipulator: Manipulator, 
        state: Optional[ManipulatorState] = None,
    ) -> bool:
        """
        Проверка столкновений манипулятора с препятствиями
        """
        if state is None:
            state = manipulator.state
        
        positions = manipulator.get_joint_positions(state)
        
        for position in positions:
            if self.is_in_collision_grid(position):
                return True
        
        rectangles = manipulator.get_link_rectangles(state)
        
        for center, length, width, angle in rectangles:
            circle_radius = np.sqrt((length/2)**2 + (width/2)**2)
            if self._quick_circle_check(center, circle_radius) \
                and self.is_in_collision_rectangle(center, length, width, angle):
                return True
        
        return False
    
    def _quick_circle_check(
            self, 
            center: np.ndarray, 
            radius: float
        ) -> bool:

        for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            
            if self.is_in_collision_grid(np.array([x, y])):
                return True
        return False