import heapq
from typing import List, Tuple, Dict, Optional


def a_star_search(
    grid: List[List[int]], start: Tuple[int, int], goal: Tuple[int, int]
) -> List[Tuple[int, int]]:
    """
    A* Pathfinding Algorithm with type hints.
    :param grid: 2D list representing the grid (0: passable, 1: obstacle)
    :param start: tuple (x, y) representing the start position
    :param goal: tuple (x, y) representing the goal position
    :return: List of tuples representing the shortest path or an empty list if no path found
    """

    # Heuristic function: Manhattan distance
    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # Directions for moving up, down, left, right
    directions: List[Tuple[int, int]] = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    # Priority queue for open set
    open_set: List[Tuple[int, Tuple[int, int]]] = []
    heapq.heappush(open_set, (0, start))  # (priority, node)

    # Dictionaries to store cost and path
    g_score: Dict[Tuple[int, int], int] = {start: 0}
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

    while open_set:
        # Get node with the lowest f(n) value
        _, current = heapq.heappop(open_set)

        # Check if goal is reached
        if current == goal:
            # Reconstruct path
            path: List[Tuple[int, int]] = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return reversed path

        # Explore neighbors
        for dx, dy in directions:
            neighbor: Tuple[int, int] = (current[0] + dx, current[1] + dy)

            # Check if neighbor is within grid bounds and passable
            if (
                0 <= neighbor[0] < len(grid)
                and 0 <= neighbor[1] < len(grid[0])
                and grid[neighbor[0]][neighbor[1]] == 0
            ):
                # Tentative g score
                tentative_g_score: int = (
                    g_score[current] + 1
                )  # Assume uniform cost for each move

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    f_score: int = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    came_from[neighbor] = current

    # No path found
    return []


# Example usage
if __name__ == "__main__":
    # Define a grid (0: passable, 1: obstacle)
    grid: List[List[int]] = [
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ]

    start: Tuple[int, int] = (0, 0)  # Starting position
    goal: Tuple[int, int] = (4, 4)  # Goal position

    path: List[Tuple[int, int]] = a_star_search(grid, start, goal)

    if path:
        print("Path found:", path)
    else:
        print("No path found.")
