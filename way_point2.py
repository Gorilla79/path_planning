import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
from scipy.spatial.distance import cdist

# File path
grid_file_path = "result_grid_test_size_min.csv"

# Manual waypoints for guidance (row, column)
manual_waypoints = [
    (156, 367),
    (205, 28),
    (239, 27),
    (94, 305),
    (96, 388),
    (193, 347),
    (225, 409),
    (27, 789),
    (67, 801)
]

def refine_waypoints(grid_array, manual_waypoints, num_waypoints=10):
    """
    Refine waypoint generation using manual waypoints as guidance.
    :param grid_array: Binary occupancy grid map.
    :param manual_waypoints: List of manually specified waypoints.
    :param num_waypoints: Desired number of waypoints.
    :return: Refined waypoints.
    """
    # Compute the distance map (distance from walls)
    distance_map = distance_transform_edt(grid_array)
    max_distance = distance_map.max()

    # Combine manual waypoints with clustering
    waypoint_candidates = np.argwhere(distance_map > np.percentile(distance_map, 80))
    all_candidates = np.vstack([manual_waypoints, waypoint_candidates])

    # Perform weighted clustering
    centroids = []
    for manual in manual_waypoints:
        # Find nearest points to manual waypoints
        distances = cdist([manual], all_candidates)
        closest_idx = np.argmin(distances, axis=1)[0]
        centroids.append(all_candidates[closest_idx])

    # Select additional waypoints if needed
    if len(centroids) < num_waypoints:
        remaining_candidates = np.delete(all_candidates, [tuple(c) for c in centroids], axis=0)
        for _ in range(num_waypoints - len(centroids)):
            centroid = remaining_candidates[np.random.randint(remaining_candidates.shape[0])]
            centroids.append(centroid)

    return np.array(centroids)

# Check if file exists
if not os.path.exists(grid_file_path):
    print(f"File not found: {grid_file_path}")
else:
    try:
        # Load the CSV file
        grid_data = pd.read_csv(grid_file_path, header=None)
        if grid_data.empty:
            raise ValueError("The CSV file is empty or improperly formatted.")

        # Replace missing or invalid values with 0 and ensure valid values
        grid_data_cleaned = grid_data.fillna(0).applymap(lambda x: 0 if x not in [0, 1] else x)
        grid_array = grid_data_cleaned.to_numpy()

        # Refine waypoints
        refined_waypoints = refine_waypoints(grid_array, manual_waypoints, num_waypoints=10)
        print("Refined waypoints generated successfully.")

        # Visualize the map with waypoints
        plt.figure(figsize=(12, 12))
        plt.imshow(grid_array, cmap='gray', origin='upper')
        plt.scatter(refined_waypoints[:, 1], refined_waypoints[:, 0], color='red', edgecolor='black', s=100, label="Refined Waypoints")
        plt.scatter([wp[1] for wp in manual_waypoints], [wp[0] for wp in manual_waypoints], color='blue', edgecolor='black', s=100, label="Manual Waypoints")
        plt.title("Refined Navigation Waypoints")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.show()

        # Print refined waypoints
        print("Refined Waypoints (row, column):")
        for idx, waypoint in enumerate(refined_waypoints, 1):
            print(f"Waypoint {idx}: {tuple(waypoint)}")

    except ValueError as ve:
        print(f"Value error: {ve}")
    except Exception as e:
        print(f"Error loading or processing CSV file: {e}")