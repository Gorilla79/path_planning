import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt


# File path
grid_file_path = "result_grid_test_size_min.csv"

# Manually defined waypoints for guidance (row, column)
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

def generate_aligned_waypoints(grid_array, manual_waypoints, num_waypoints=9):
    """
    Generate waypoints closely aligned with manually defined waypoints.
    :param grid_array: Binary grid map.
    :param manual_waypoints: List of manually specified waypoints.
    :param num_waypoints: Desired number of waypoints.
    :return: List of refined waypoints.
    """
    distance_map = distance_transform_edt(grid_array)  # Distance from walls
    max_distance = distance_map.max()

    # Initialize refined waypoints
    refined_waypoints = []

    # Rule: Align to manual waypoints
    for manual_wp in manual_waypoints:
        # Find the closest valid point on the distance map
        distances = np.linalg.norm(np.argwhere(grid_array == 1) - np.array(manual_wp), axis=1)
        nearest_point = np.argwhere(grid_array == 1)[np.argmin(distances)]
        refined_waypoints.append(nearest_point)

    # Ensure unique waypoints
    refined_waypoints = np.unique(refined_waypoints, axis=0)

    return np.array(refined_waypoints)

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

        # Generate aligned waypoints
        aligned_waypoints = generate_aligned_waypoints(grid_array, manual_waypoints, num_waypoints=9)

        # Visualize the map with aligned waypoints
        plt.figure(figsize=(12, 12))
        plt.imshow(grid_array, cmap='gray', origin='upper')
        plt.scatter(aligned_waypoints[:, 1], aligned_waypoints[:, 0], color='red', edgecolor='black', s=100, label="Aligned Waypoints")
        plt.scatter([wp[1] for wp in manual_waypoints], [wp[0] for wp in manual_waypoints], color='blue', edgecolor='black', s=100, label="Manual Waypoints")
        plt.title("Aligned Navigation Waypoints")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.show()

        # Print aligned waypoints
        print("Aligned Waypoints (row, column):")
        for idx, waypoint in enumerate(aligned_waypoints, 1):
            print(f"Waypoint {idx}: {tuple(waypoint)}")

    except ValueError as ve:
        print(f"Value error: {ve}")
    except Exception as e:
        print(f"Error loading or processing CSV file: {e}")