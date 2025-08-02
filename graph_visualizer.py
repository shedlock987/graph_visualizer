import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

# Get the current directory of the script
current_dir = os.path.dirname(os.path.abspath(__file__))
# Get the parent directory
parent_dir = os.path.dirname(current_dir)
# append rrt_graph_builder to parent directory
rrt_graph_builder_dir = os.path.join(parent_dir, 'rrt_graph_builder/rrtDemo')
# Check if the directory exists
if os.path.exists(rrt_graph_builder_dir):
    print(f"Adding {rrt_graph_builder_dir} to sys.path")
else:
    print(f"Directory {rrt_graph_builder_dir} does not exist. Dependency missing.")
# Add the parent directory to sys.path
sys.path.append(rrt_graph_builder_dir)

import rrtDemo  # Importing rrtDemo from rrt_graph_builder
#from rrtDemo import RRT
rrt = RRT(-5, 0, 5, 5, 0, 0, 5, 5, 1.05, 1.0, 0.5, 2.0, 10.0, False, 10000)  # Create an instance of the RRT class



# Array creation
arr1 = np.array([1, 2, 3, 4, 5]) # Creating a 1D array
arr2 = np.array([[1, 2, 3], [4, 5, 6]]) # Creating a 2D array

# Basic operations
arr3 = arr1 + 2 # Adding 2 to each element of arr1
arr4 = arr1 * arr1 # Element-wise multiplication
arr5 = np.sum(arr1) # Sum of all elements in arr1

# Indexing
element1 = arr1[0] # Accessing the first element of arr1
row1 = arr2[0] # Accessing the first row of arr2
element2 = arr2[1, 2] # Accessing the element at row 1, column 2 of arr2

# Conditional selection
arr6 = arr1[arr1 > 3] # Selecting elements of arr1 greater than 3

print("Array 1:", arr1)
print("Array 2:", arr2)
print("Array 3 (arr1 + 2):", arr3)
print("Array 4 (arr1 * arr1):", arr4)
print("Sum of Array 1:", arr5)
print("First element of Array 1:", element1)
print("First row of Array 2:", row1)
print("Element at row 1, column 2 of Array 2:", element2)
print("Elements of Array 1 greater than 3:", arr6)

# Sample data for nodes (x, y, z coordinates)
nodes = np.array([
    [1, 1, 1],
    [2, 3, 2],
    [3, 1, 4],
    [4, 2, 3],
    [5, 3, 1]
])

# Define connections (edges) between nodes
edges = [
    (0, 1),  # Node 0 to Node 1
    (0, 2),  # Node 0 to Node 2
    (1, 3),  # Node 1 to Node 3
    (2, 4),  # Node 2 to Node 4
    (3, 4)   # Node 3 to Node 4
]

# Create the 3D plot
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Plot nodes as scatter points
ax.scatter(nodes[:, 0], nodes[:, 1], nodes[:, 2], c='blue', marker='o', s=100)

# Plot edges as lines
for i, j in edges:
    x_coords = [nodes[i, 0], nodes[j, 0]]
    y_coords = [nodes[i, 1], nodes[j, 1]]
    z_coords = [nodes[i, 2], nodes[j, 2]]
    ax.plot(x_coords, y_coords, z_coords, c='gray')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('time (s)')

# Show the plot
plt.show()