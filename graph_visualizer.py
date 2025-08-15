import sys
import os
import numpy as np
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


# Build the RRT tree
vis_rrt.buildRRT()

# Check results
print("RRT build complete:", vis_rrt.isComplete())
print("Number of nodes:", vis_rrt.getNodeCount())

# Alternatively, build step by step
# vis_rrt = display_RRT.RRT()
# vis_rrt.initializeRRT(...)  # same as above
# vis_rrt.setOccupancyMap(...)  # same as above
# while not vis_rrt.isComplete():
#     stepped = vis_rrt.stepRRT()
#     print("Stepped:", stepped)
# print("Final node count:", vis_rrt.getNodeCount())