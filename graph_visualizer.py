import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

sys.path.append(os.path.join(os.path.dirname(__file__), '../rrt_graph_builder/rrtDemo'))
import rrtDemo

# Example usage of the exposed VisRRT class (named RRT in Python)

# Initialize with default constructor
vis_rrt = rrtDemo.RRT()

# Set up parameters for the RRT
range_a_x = -5.0
range_a_y = 0.0
range_b_x = 5.0
range_b_y = 5.0
origin_x = 0.0
origin_y = 0.0
dest_x = 4.5
dest_y = 4.5
max_angle_rad = 0.8
max_dist = 1.0
min_dist = 0.5
max_interval = 2.0
max_time = 10.0
dim_3D = False
node_limit = 10000

vis_rrt.initializeRRT(
    range_a_x, range_a_y,
    range_b_x, range_b_y,
    origin_x, origin_y,
    dest_x, dest_y,
    max_angle_rad, max_dist,
    min_dist, max_interval,
    max_time, dim_3D, node_limit
)

# Define a simple occupancy map (example obstacles)
occp_coords = [[float(x), float(y)] for x, y in [[1.0, 1.0], [2.5, 2.5], [3.0, 1.5]]]
occp_widths = [float(w) for w in [0.5, 0.6, 0.4]]
occp_interval = [float(i) for i in [1.0, 1.0, 1.0]]

vis_rrt.setOccupancyMap(occp_coords, occp_widths, occp_interval)


# Build the RRT tree step by step
while not vis_rrt.isComplete():
    vis_rrt.stepRRT()

# Check results
print("RRT build complete:", vis_rrt.isComplete())
print("Number of nodes:", vis_rrt.getNodeCount())

# Create the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot each occupancy grid cell as a rectangular prism (assuming square base in XY, height along Z)
for i in range(len(occp_coords)):
    x = occp_coords[i][0]
    y = occp_coords[i][1]
    w = occp_widths[i]  # width (side length in X and Y)
    h = occp_interval[i]  # height (along Z)
    # bar3d(x, y, z, dx, dy, dz) where (x,y,z) is the bottom-left corner
    ax.bar3d(x - w / 2, y - w / 2, 0, w, w, h, shade=True, color='red', alpha=0.8)

# Plot a blue dot at the origin (0,0,0)
ax.scatter(0, 0, 0, color='blue', s=50)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')
ax.set_title('3D Plot of Occupancy Grid Cells')

# Set axis limits
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_zlim(0, 10)

# Display the plot
plt.show()