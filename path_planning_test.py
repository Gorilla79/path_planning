import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop
from scipy.spatial import distance

# A* 알고리즘 정의
def heuristic(a, b):
    return distance.euclidean(a, b)

def astar_path(grid, start, goal):
    rows, cols = grid.shape
    open_set = []
    heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        _, current_cost, current = heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] == 1:
                new_cost = current_cost + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heappush(open_set, (priority, new_cost, neighbor))
                    came_from[neighbor] = current
    return []

# 그리드 맵 데이터 (0: 벽, 1: 경로)
grid_array = np.loadtxt('result_grid_test_size_min.csv', delimiter=',')

# 웨이포인트 정의
waypoints_coordinates = {
    1: (156, 367),
    2: (205, 28),
    3: (239, 27),
    4: (94, 305),
    5: (96, 388),
    6: (193, 347),
    7: (225, 409),
    8: (27, 789),
    9: (67, 801)
}

# 경로 계산
paths = []
failed_paths = []
waypoints_list = list(waypoints_coordinates.values())
for i in range(len(waypoints_list) - 1):
    start = waypoints_list[i]
    goal = waypoints_list[i + 1]
    path = astar_path(grid_array, start, goal)
    if path:
        paths.append(path)
    else:
        failed_paths.append((i + 1, start, goal))

# 경로 시각화
colors = ['red', 'orange', 'yellow', 'lightgreen', 'green', 'cyan', 'blue', 'darkblue']
plt.figure(figsize=(12, 12))
plt.imshow(grid_array, cmap='gray', origin='upper')

# 경로 그리기
for idx, path in enumerate(paths):
    path_x, path_y = zip(*path)
    plt.plot(path_y, path_x, color=colors[idx], label=f"Path {idx+1}")

# 웨이포인트 표시
for wp, coord in waypoints_coordinates.items():
    plt.scatter(coord[1], coord[0], color='white', edgecolor='black', s=100, label=f"Waypoint {wp}")

plt.title("Grid Map with Shortest Paths Between Waypoints")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.show()

# 실패한 경로 출력
if failed_paths:
    print("Failed to generate paths for the following waypoint pairs:")
    for idx, start, goal in failed_paths:
        print(f"Path {idx}: Start {start} -> Goal {goal}")