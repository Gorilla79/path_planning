import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent

# File path
grid_file_path = "result_grid_test_size_min.csv"

# List to store manually clicked waypoints
clicked_waypoints = []

def onclick(event: MouseEvent):
    """
    Event handler for mouse clicks to select waypoints.
    """
    if event.xdata is not None and event.ydata is not None:
        # Get the x and y coordinates of the click
        x, y = int(event.xdata), int(event.ydata)
        clicked_waypoints.append((int(y), int(x)))  # Store as (row, column)
        
        # Display the waypoint number on the map
        waypoint_number = len(clicked_waypoints)
        ax.scatter(x, y, color='red', edgecolor='black', s=100)  # Plot the point
        ax.text(x, y, f"{waypoint_number}", color='blue', fontsize=12, ha='center', va='center')  # Label it
        print(f"Waypoint {waypoint_number}: (row={int(y)}, column={int(x)})")
        plt.draw()

        # Stop after 9 waypoints are selected
        if len(clicked_waypoints) >= 9:
            print("\n9 waypoints selected. Closing the map.")
            plt.close()

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

        # Plot the grid map
        fig, ax = plt.subplots(figsize=(12, 12))
        ax.imshow(grid_array, cmap='gray', origin='upper')
        ax.set_title("Click to Select 9 Waypoints")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        plt.connect('button_press_event', onclick)  # Connect the click event
        plt.show()

        # Print the selected waypoints
        print("\nSelected Waypoints (row, column):")
        for idx, waypoint in enumerate(clicked_waypoints, 1):
            print(f"Waypoint {idx}: {waypoint}")

    except ValueError as ve:
        print(f"Value error: {ve}")
    except Exception as e:
        print(f"Error loading or processing CSV file: {e}")