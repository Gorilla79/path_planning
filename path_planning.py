import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
from heapq import heappush, heappop
from scipy.spatial import distance

# Load the grid map from the CSV file
grid_file_path = "result_grid_test_size_min.csv"  # Update with your CSV file path
grid_array = pd.read_csv(grid_file_path, header=None).to_numpy()

# Define the heuristic function for A* (Euclidean distance)
def heuristic(a, b):
    return distance.euclidean(a, b)

# Adjusted A* algorithm to incorporate weighted grid
def astar_weighted(grid, weights, start, goal):
    rows, cols = grid.shape
    open_set = []
    heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        _, current_cost, current = heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return reversed path

        # Explore neighbors
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-connectivity
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] == 1:
                new_cost = current_cost + weights[neighbor]
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heappush(open_set, (priority, new_cost, neighbor))
                    came_from[neighbor] = current

    return []  # No path found

# Input waypoints (row, column) manually based on the CSV grid
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

# Compute the distance transform (distance from walls)
distance_map = distance_transform_edt(grid_array)

# Normalize the distance map to use as weights (higher = farther from walls)
max_distance = distance_map.max()
weighted_grid = max_distance - distance_map  # Invert for pathfinding (lower cost = farther from walls)

# Generate paths using the adjusted A* algorithm
waypoints_list = list(waypoints_coordinates.values())
adjusted_paths = []
failed_adjusted_paths = []

for i in range(len(waypoints_list) - 1):
    start = waypoints_list[i]
    goal = waypoints_list[i + 1]
    path = astar_weighted(grid_array, weighted_grid, start, goal)
    if path:
        adjusted_paths.append(path)
    else:
        failed_adjusted_paths.append((i + 1, start, goal))

# Visualization of the grid map with adjusted paths
colors = ['red', 'orange', 'yellow', 'lightgreen', 'green', 'cyan', 'blue', 'darkblue']
plt.figure(figsize=(12, 12))
plt.imshow(grid_array, cmap='gray', origin='upper')

# Plot the adjusted paths
for idx, path in enumerate(adjusted_paths):
    path_x, path_y = zip(*path)
    plt.plot(path_y, path_x, color=colors[idx], label=f"Path {idx+1} (Adjusted)")

# Overlay the waypoints
for wp, coord in waypoints_coordinates.items():
    plt.scatter(coord[1], coord[0], color='white', edgecolor='black', s=100, label=f"Waypoint {wp}")

# Add titles and legend
plt.title("Grid Map with Adjusted Paths (Maintaining Distance from Walls)")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.show()

# Print failed adjusted paths for debugging
if failed_adjusted_paths:
    print("Failed to generate adjusted paths for the following waypoint pairs:")
    for idx, start, goal in failed_adjusted_paths:
        print(f"Path {idx}: Start {start} -> Goal {goal}")