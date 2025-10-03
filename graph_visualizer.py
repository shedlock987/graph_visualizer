import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
sys.path.append(os.path.join(os.path.dirname(__file__), '../rrt_graph_builder/rrtDemo'))
import rrtDemo
# Enable/disable GIF rendering
render_gif = False
# Example usage of the exposed VisRRT class (named RRT in Python)
# Set up parameters for the RRT
range_a_x = -5.0
range_a_y = -5.0
range_b_x = 5.0
range_b_y = 5.0
origin_x = 0.0
origin_y = 0.0
origin_time = 0.0 # Time for origin
dest_x = 4.5
dest_y = 4.5
dest_time = 10.0 # Time for destination (e.g., max_time)
max_angle_rad = 0.1
max_dist = 2.0
min_dist = 0.5
max_interval = 1.5
max_time = 10.0
dim_3D = True
iteration_limit = 5000
initial_heading = 0.785
# Use tuples (pose_t) for ranges, origin, dest
range_a = (range_a_x, range_a_y, 0.0, 0.0) # Time=0 for range_a (ignored in extraction)
range_b = (range_b_x, range_b_y, 0.0, 0.0) # Time=0 for range_b
origin = (origin_x, origin_y, origin_time, initial_heading)
dest = (dest_x, dest_y, dest_time, 0.0)

# Define occupancy map as a list of ((x, y, time, heading), width) tuples
occupancy_map = [
    ((1.0, 1.0, 2.0, 0.0), 0.5),
    ((2.5, 2.5, 0.5, 0.0), 2.0),
    ((3.0, 1.5, 5.0, 0.0), 0.4)
]

vis_rrt = rrtDemo.RRT(occupancy_map, range_a, range_b, origin, dest,
                      max_angle_rad, max_dist,
                      min_dist, max_interval,
                      max_time, dim_3D, iteration_limit, initial_heading)

vis_rrt.updateInitialHeading(initial_heading)

# Build the RRT tree step by step
while not vis_rrt.isComplete():
    vis_rrt.stepRRT()
# Check results
print("RRT build complete:", vis_rrt.isComplete())
print("Number of nodes:", vis_rrt.getNodeCount() - 2) # Exclude origin and dest
# Collect node positions and forward connections
node_count = vis_rrt.getNodeCount()
all_node_xs = []
all_node_ys = []
all_node_zs = []
fwd_dict = {}
for i in range(node_count):
    node = vis_rrt.getNodeAt(i)
    if node:
        x = node.xCrdnt()
        y = node.yCrdnt()
        t = node.time()
        all_node_xs.append(x)
        all_node_ys.append(y)
        all_node_zs.append(t)
    fwd_indices = vis_rrt.getForwardIndices(i)
    fwd_dict[i] = list(fwd_indices)
num_frames = len(all_node_xs)
duration_ms = 15000 # 15 seconds
interval_ms = max(1, duration_ms // num_frames) # At least 1 ms per frame
# Render GIF animation if enabled
if render_gif:
    # Create the 3D figure for animation (larger size)
    fig_anim = plt.figure(figsize=(12.8, 9.6))
    ax_anim = fig_anim.add_subplot(111, projection='3d')
    # Static elements data (for replotting in each frame)
    obstacle_data = []

    for occ in occupancy_map:
        (x, y, z, heading), w = occ
        h = max_time  # or use a fixed height if you want, or z if you want to use the time from the tuple
        obstacle_data.append((x - w / 2, y - w / 2, 0, w, w, h))
    def animate(frame):
        ax_anim.cla()
        # Replot obstacles
        for x_start, y_start, z_start, dx, dy, dz in obstacle_data:
            ax_anim.bar3d(x_start, y_start, z_start, dx, dy, dz, shade=True, color='red', alpha=0.8)

        # Replot origin and destination
        ax_anim.scatter(0, 0, 0, color='blue', s=50)

        # Plot a vertical green line at the destination (orthogonal to z/time plane)
        ax_anim.plot([dest_x, dest_x], [dest_y, dest_y], [0, dest_time], color='green', linewidth=3)
        # Plot nodes up to current frame
        current_xs = all_node_xs[:frame + 1]
        current_ys = all_node_ys[:frame + 1]
        current_zs = all_node_zs[:frame + 1]
        if current_xs:
            ax_anim.scatter(current_xs, current_ys, current_zs, color='orange', s=20)
        # Draw blue lines for forward connections where both nodes are visible
        for i in range(frame + 1):
            fwd_list = fwd_dict.get(i, []) # Use cached
            x1, y1, z1 = all_node_xs[i], all_node_ys[i], all_node_zs[i]
            for fwd_idx in fwd_list:
                if fwd_idx <= frame: # Only draw if forward node is already added
                    x2, y2, z2 = all_node_xs[fwd_idx], all_node_ys[fwd_idx], all_node_zs[fwd_idx]
                    ax_anim.plot([x1, x2], [y1, y2], [z1, z2], color='blue', linewidth=1, alpha=0.7)
        # Set labels, title, limits
        ax_anim.set_xlabel('X')
        ax_anim.set_ylabel('Y')
        ax_anim.set_zlabel('Time')
        ax_anim.set_title('3D Animation of RRT Node Placement')
        ax_anim.set_xlim(-6, 6)
        ax_anim.set_ylim(-6, 6)
        ax_anim.set_zlim(0, 12)
    # Create and save the animation as GIF
    anim = animation.FuncAnimation(fig_anim, animate, frames=num_frames, interval=interval_ms, blit=False, repeat=True)
    anim.save('rrt_animation.gif', writer='pillow')
# Create the static 3D plot (larger size)
fig = plt.figure(figsize=(19.2, 14.4))
ax = fig.add_subplot(111, projection='3d')

# Plot each occupancy grid cell as a rectangular prism (assuming square base in XY, height along Z)
for occ in occupancy_map:
    (x, y, z, heading), w = occ
    h = z  # Use the z (time) value from the tuple as the height of the prism
    ax.bar3d(x - w / 2, y - w / 2, 0, w, w, h, shade=True, color='red', alpha=0.8)
# Plot a blue dot at the origin (0,0,0)
ax.scatter(0, 0, 0, color='blue', s=50)
# Plot a vertical green line at the destination (orthogonal to z/time plane)
ax.plot([dest_x, dest_x], [dest_y, dest_y], [0, dest_time], color='green', linewidth=3)
# Plot orange points for each node in the graph
node_xs = all_node_xs # Reuse
node_ys = all_node_ys
node_zs = all_node_zs
if node_xs: # Only scatter if there are nodes
    ax.scatter(node_xs, node_ys, node_zs, color='orange', s=20)
# Draw blue lines for all forward connections
for i in range(node_count):
    fwd_list = fwd_dict.get(i, [])
    x1, y1, z1 = node_xs[i], node_ys[i], node_zs[i]
    for fwd_idx in fwd_list:
        if fwd_idx < node_count: # Ensure valid index
            x2, y2, z2 = node_xs[fwd_idx], node_ys[fwd_idx], node_zs[fwd_idx]
            ax.plot([x1, x2], [y1, y2], [z1, z2], color='blue', linewidth=1, alpha=0.7)
# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')
ax.set_title('3D Plot of Occupancy Grid Cells and RRT Nodes')
# Set axis limits (adjusted for the range -5 to 5)
ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.set_zlim(0, 12)
# Display the plot
plt.show()
# Count how many nodes share the same x, y as the destination (within 1% error)
xy_tol = 0.05 # 1% tolerance
count_at_dest_xy = 0
for i in range(node_count):
    node = vis_rrt.getNodeAt(i)
    if node:
        x = node.xCrdnt()
        y = node.yCrdnt()
        # Check if x and y are within xy_tol% of dest_x and dest_y
        if abs(x - dest_x) <= abs(dest_x) * xy_tol and abs(y - dest_y) <= abs(dest_y) * xy_tol:
            count_at_dest_xy += 1

print(f"Number of nodes at (x, y) ≈ ({dest_x}, {dest_y}) within 1% error: {count_at_dest_xy}")

