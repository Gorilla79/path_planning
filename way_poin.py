import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
from scipy.spatial.distance import cdist
from sklearn.cluster import KMeans

# File path
grid_file_path = "result_grid_test_size_min.csv"

# Custom KMeans function as a fallback
def custom_kmeans(data, k, max_iters=100, tol=1e-4):
    """
    Custom KMeans implementation for clustering.
    :param data: Input data points (n_samples, n_features).
    :param k: Number of clusters.
    :param max_iters: Maximum iterations for clustering.
    :param tol: Tolerance for centroid changes.
    :return: Final centroids and cluster assignments.
    """
    np.random.seed(0)
    centroids = data[np.random.choice(data.shape[0], k, replace=False)]

    for _ in range(max_iters):
        # Assign points to nearest centroid
        distances = cdist(data, centroids, metric='euclidean')
        labels = np.argmin(distances, axis=1)

        # Recompute centroids
        new_centroids = np.array([data[labels == i].mean(axis=0) for i in range(k)])
        if np.all(np.abs(new_centroids - centroids) < tol):
            break
        centroids = new_centroids

    return centroids, labels

# Test whether KMeans from sklearn works
def test_sklearn_kmeans():
    test_data = np.array([[1, 2], [1, 4], [1, 0],
                          [4, 2], [4, 4], [4, 0]])
    try:
        print("Testing sklearn KMeans...")
        kmeans = KMeans(n_clusters=2, random_state=0, n_init=10)
        kmeans.fit(test_data)
        print("Sklearn KMeans test successful!")
    except Exception as e:
        print(f"Sklearn KMeans test failed: {e}")
        return False
    return True

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

        # Compute the distance map
        distance_map = distance_transform_edt(grid_array)
        print(f"Distance map max value: {distance_map.max()}")

        # Find waypoint candidates
        percentile_threshold = 90
        waypoint_candidates = np.argwhere(distance_map > np.percentile(distance_map, percentile_threshold))
        print(f"Found {waypoint_candidates.shape[0]} waypoint candidates at {percentile_threshold}th percentile.")

        # Validate waypoint candidates
        if waypoint_candidates.size == 0:
            raise ValueError("No valid waypoint candidates found after adjustments.")
        print(f"waypoint_candidates.shape: {waypoint_candidates.shape}")
        print(f"waypoint_candidates example: {waypoint_candidates[:5]}")

        # Run clustering
        num_waypoints = 10
        if waypoint_candidates.shape[0] < num_waypoints:
            raise ValueError(
                f"Not enough waypoint candidates for {num_waypoints} clusters. "
                f"Found only {waypoint_candidates.shape[0]} candidates."
            )

        print("Running KMeans clustering...")
        use_sklearn = test_sklearn_kmeans()  # Check if sklearn KMeans works

        if use_sklearn:
            # Sklearn KMeans
            kmeans = KMeans(n_clusters=num_waypoints, random_state=0, n_init=10, max_iter=300)
            kmeans.fit(waypoint_candidates)
            waypoints = kmeans.cluster_centers_.astype(int)
            print("Sklearn KMeans clustering completed successfully.")
        else:
            # Fallback: Custom KMeans
            waypoints, _ = custom_kmeans(waypoint_candidates, k=num_waypoints)
            waypoints = waypoints.astype(int)
            print("Custom KMeans clustering completed successfully.")

        # Visualize the map with waypoints
        plt.figure(figsize=(12, 12))
        plt.imshow(grid_array, cmap='gray', origin='upper')
        plt.scatter(waypoints[:, 1], waypoints[:, 0], color='red', edgecolor='black', s=100, label="Generated Waypoints")
        plt.title("Generated Navigation Waypoints")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.show()

        # Print waypoints
        print("Generated Waypoints (row, column):")
        for idx, waypoint in enumerate(waypoints, 1):
            print(f"Waypoint {idx}: {tuple(waypoint)}")

    except ValueError as ve:
        print(f"Value error: {ve}")
    except Exception as e:
        print(f"Error loading or processing CSV file: {e}")
