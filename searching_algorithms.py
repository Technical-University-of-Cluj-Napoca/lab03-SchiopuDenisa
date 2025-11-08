import math
from utils import *
from collections import deque
from queue import PriorityQueue
from grid import Grid
from spot import Spot

def bfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Breadth-First Search (BFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    if start is None or end is None:
        return False

    queue = deque()
    queue.append(start)

    visited = {start}
    came_from = {}
    while queue:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False

        current = queue.popleft()

        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)
                neighbor.make_open()
        draw()

        if current != start:
            current.make_closed()
    return False

def dfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Depdth-First Search (DFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    if start is None or end is None:
        return False

    stack = [start]
    visited = {start}
    came_from = {}
    while stack:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False

        current = stack.pop()
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                stack.append(neighbor)
                neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False

def h_manhattan_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def h_euclidian_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def reconstruct_path(came_from: dict, current: Spot, draw: callable) -> None:
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()

def astar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    A* Pathfinding Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}

    g_score = {spot: float("inf") for row in grid.grid for spot in row}
    g_score[start] = 0

    f_score = {spot: float("inf") for row in grid.grid for spot in row}
    f_score[start] = h_manhattan_distance(start.get_position(), end.get_position())

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            temp_g = g_score[current] + 1  # uniform cost for each move

            if temp_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g
                f_score[neighbor] = temp_g + h_manhattan_distance(
                    neighbor.get_position(), end.get_position()
                )

                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False

def uniform_cost_search(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))  # (cost, tie-breaker, node)
    came_from = {}

    g_score = {spot: float("inf") for row in grid.grid for spot in row}
    g_score[start] = 0

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            new_cost = g_score[current] + 1

            if new_cost < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = new_cost

                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((new_cost, count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False

def depth_limited_search(draw: callable, grid: Grid, start: Spot, end: Spot, limit: int) -> bool:

    def recursive_dls(current: Spot, depth: int, came_from: dict, visited: set) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            return True

        if depth >= limit:
            return False

        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                neighbor.make_open()
                draw()

                if recursive_dls(neighbor, depth + 1, came_from, visited):
                    return True
                neighbor.make_closed()
                draw()
        return False

    came_from = {}
    visited = {start}
    return recursive_dls(start, 0, came_from, visited)

def dijkstra(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:

    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid.grid for spot in row}
    g_score[start] = 0
    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            temp_g = g_score[current] + 1

            if temp_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g

                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((g_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()
    return False

def iterative_deepening_dfs(draw: callable, grid: Grid, start: Spot, end: Spot, max_depth: int = 100) -> bool:
    for depth in range(max_depth + 1):
        if depth_limited_search(draw, grid, start, end, depth):
            return True
    return False

def ida_star(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:

    def search(current: Spot, g: float, bound: float, came_from: dict, visited: set):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        f = g + h_manhattan_distance(current.get_position(), end.get_position())

        if f > bound:
            return f

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            return True

        min_threshold = float("inf")

        for neighbor in current.neighbors:
            if neighbor.is_barrier() or neighbor in visited:
                continue

            visited.add(neighbor)
            came_from[neighbor] = current
            neighbor.make_open()
            draw()
            temp = search(neighbor, g + 1, bound, came_from, visited)

            if temp is True:
                return True

            if isinstance(temp, (int, float)) and temp < min_threshold:
                min_threshold = temp

            neighbor.make_closed()
            draw()
            visited.remove(neighbor)
        return min_threshold

    bound = h_manhattan_distance(start.get_position(), end.get_position())
    came_from = {}

    while True:
        visited = {start}
        temp = search(start, 0, bound, came_from, visited)
        if temp is True:
            return True
        if temp == float("inf"):
            return False
        bound = temp