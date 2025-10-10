import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import matplotlib.image as mpimg
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
origin_x = -5.0
origin_y = 0.0
origin_time = 0.0 # Time for origin
dest_x = 4.5
dest_y = 4.5
dest_time = 10.0 # Time for destination (e.g., max_time)
max_angle_rad = 0.3
max_dist = 2.0
min_dist = 0.5
max_interval = 1.5
max_time = 10.0
dim_3D = True
iteration_limit = 500
initial_heading = 0.78 # Initial heading in radians (e.g., 45 degrees)
max_admissible = 7
# Use tuples (pose_t) for ranges, origin, dest
range_a = (range_a_x, range_a_y, 0.0, 0.0) # Time=0 for range_a (ignored in extraction)
range_b = (range_b_x, range_b_y, max_time, 0.0) # Time=0 for range_b
origin = (origin_x, origin_y, origin_time, initial_heading)
dest = (dest_x, dest_y, dest_time, 0.0)
# Define occupancy map as a list of ((x, y, time, heading), width) tuples
# Add boxes with time (z) determined by the slope, width=0.5, height=1, centroid along the slope line
# x(y) = -2, y in [-5, 5], 3x density, slope of 16, height of 1
occupancy_map = [
    ((1.0, 1.0, 12.0, 0.0), 0.5),
    ((1.0, 4.0, 12.0, 0.0), 0.4)
]
for y in np.linspace(-5, 5, num=33):  # 33 boxes for 3x density
    z_centroid = 1.0 + 8 * (y + 5) / 10  # slope line for centroid, slope reduced from 12 to 8
    occupancy_map.append(((-2.0, y, z_centroid, 0.0), 0.5))  # z_centroid is the center, box height is always 1

vis_rrt = rrtDemo.RRT(
    occupancy_map, range_a, range_b, origin, dest,
    max_angle_rad, max_dist,
    min_dist, max_interval,
    max_time, dim_3D, iteration_limit,
    max_admissible  # <-- add this argument
)

#vis_rrt.updateInitialHeading(initial_heading)
# Build the RRT tree step by step
while not vis_rrt.isComplete():
    vis_rrt.stepRRT()
# Check results
print("RRT build complete:", vis_rrt.isComplete())
print("Number of nodes:", vis_rrt.getNodeCount() - 2) # Exclude origin and dest

# Get the coordinates of the first node in vis_rrt
first_node = vis_rrt.getNodeAt(0)
if first_node:
    first_x = first_node.xCrdnt()
    first_y = first_node.yCrdnt()
    first_z = first_node.time()
else:
    first_x, first_y, first_z = 0, 0, 0  # fallback if node not found

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
# Get the last node and check admissibility
last_node = vis_rrt.getNodeAt(node_count - 1)
if last_node:
    last_x = last_node.xCrdnt()
    last_y = last_node.yCrdnt()
    last_z = last_node.time()
    admissible = vis_rrt.isAdmissible(last_node)
else:
    last_x, last_y, last_z = None, None, None
    admissible = False

# Find all nodes at (dest_x, dest_y) within a small tolerance
xy_tol = 1e-2  # tolerance for floating point comparison
green_indices = []
for i in range(node_count):
    node = vis_rrt.getNodeAt(i)
    if node:
        x = node.xCrdnt()
        y = node.yCrdnt()
        if abs(x - dest_x) <= xy_tol and abs(y - dest_y) <= xy_tol:
            green_indices.append(i)

# Render GIF animation if enabled
if render_gif:
    # Double the size of the GIF render (was 6.4 x 4.8, now 12.8 x 9.6)
    fig_anim = plt.figure(figsize=(12.8, 9.6))
    ax_anim = fig_anim.add_subplot(111, projection='3d')
    # Static elements data (for replotting in each frame)
    obstacle_data = []
    for occ in occupancy_map:
        (x, y, z, heading), w = occ
        h = z # Use the z (time) value from the tuple as the height of the prism
        obstacle_data.append((x - w / 2, y - w / 2, 0, w, w, h))
    # Load the image
    img = mpimg.imread("rrt_plot.jpeg")  # Use your JPG filename

    # Set desired image height in plot
    img_height = 2.0  # Height in plot units
    img_width = img.shape[1] / img.shape[0] * img_height  # Scale width proportionally

    # Define the location and size for the image "billboard"
    img_x = np.linspace(-1, -1 + img_width, img.shape[1])
    img_y = np.linspace(-5, -5 + img_height, img.shape[0])
    img_x, img_y = np.meshgrid(img_x, img_y)
    img_z = np.zeros_like(img_x)  # z=0 for the bottom

    def animate(frame):
        ax_anim.cla()
        # Replot obstacles
        for x_start, y_start, z_start, dx, dy, dz in obstacle_data:
            ax_anim.bar3d(x_start, y_start, z_start, dx, dy, dz, shade=True, color='red', alpha=0.8)
        # Plot a blue dot at the first node
        ax_anim.scatter(first_x, first_y, first_z, color='blue', s=50)
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
            fwd_list = fwd_dict.get(i, [])
            x1, y1, z1 = all_node_xs[i], all_node_ys[i], all_node_zs[i]
            for fwd_idx in fwd_list:
                if fwd_idx <= frame:
                    x2, y2, z2 = all_node_xs[fwd_idx], all_node_ys[fwd_idx], all_node_zs[fwd_idx]
                    ax_anim.plot([x1, x2], [y1, y2], [z1, z2], color='blue', linewidth=1, alpha=0.7)
        # Plot large green dots for all nodes at (dest_x, dest_y) that are visible
        for idx in green_indices:
            if idx <= frame:
                ax_anim.scatter(all_node_xs[idx], all_node_ys[idx], all_node_zs[idx], color='green', s=60)
        # Set labels, title, limits
        ax_anim.set_xlabel('X')
        ax_anim.set_ylabel('Y')
        ax_anim.set_zlabel('Time')
        ax_anim.set_title('3D Animation of RRT Node Placement')
        ax_anim.set_xlim(-6, 6)
        ax_anim.set_ylim(-6, 6)
        ax_anim.set_zlim(0, 12)
        ax_anim.view_init(elev=20, azim=-160)  # <-- Rotate view 180 degrees clockwise
        ax_anim.plot_surface(img_x, img_y, img_z, rstride=1, cstride=1, facecolors=img, shade=False)
    # Create and save the animation as GIF
    anim = animation.FuncAnimation(fig_anim, animate, frames=num_frames, interval=interval_ms, blit=False, repeat=True)
    anim.save('rrt_animation.gif', writer='pillow')
# Find all nodes at (dest_x, dest_y) within a small tolerance
xy_tol = 1e-2  # tolerance for floating point comparison
green_indices = []
for i in range(node_count):
    node = vis_rrt.getNodeAt(i)
    if node:
        x = node.xCrdnt()
        y = node.yCrdnt()
        if abs(x - dest_x) <= xy_tol and abs(y - dest_y) <= xy_tol:
            green_indices.append(i)

# Create the static 3D plot (larger size)
fig = plt.figure(figsize=(9.6, 7.2))
ax = fig.add_subplot(111, projection='3d')

for occ in occupancy_map:
    (x, y, z, heading), w = occ
    # For the two vertical bars, plot from z=0 up to z=12
    if (x, y, z) == (1.0, 1.0, 12.0) or (x, y, z) == (1.0, 4.0, 12.0):
        ax.bar3d(x - w / 2, y - w / 2, 0, w, w, z, shade=True, color='red', alpha=0.8)
    else:
        # For sloped bars, use height 1 centered at z
        h = 1.0
        ax.bar3d(x - w / 2, y - w / 2, z - h / 2, w, w, h, shade=True, color='red', alpha=0.8)

# Plot a blue dot at the first node
ax.scatter(first_x, first_y, first_z, color='blue', s=50)
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
# Plot large green dots for all nodes at (dest_x, dest_y)
for idx in green_indices:
    ax.scatter(node_xs[idx], node_ys[idx], node_zs[idx], color='green', s=60)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')
ax.set_title('3D Visualization of RRT Node Placement')

# Set axis limits
ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.set_zlim(0, 12)
ax.view_init(elev=20, azim=-160)  # <-- Rotate view 180 degrees clockwise

# Load your JPG image (make sure it's not the plot you just saved)
img = mpimg.imread("Crosswalk.jpg")
if img.dtype == np.uint8:
    img = img.astype(np.float32) / 255.0

img = np.flipud(img)      # Flip vertically (upright)
img = np.fliplr(img)      # Flip horizontally (invert left to right)

img_height = 3.0
img_width = img.shape[1] / img.shape[0] * img_height

# For the yz plane, x is fixed, y and z vary
fixed_x = -2.2  # x position for the left edge of the image
img_y = np.linspace(-5, -5 + img_width, img.shape[1])
img_z = np.linspace(0, img_height, img.shape[0])
img_y, img_z = np.meshgrid(img_y, img_z)
img_x = np.full_like(img_y, fixed_x)

ax.plot_surface(img_x, img_y, img_z, rstride=1, cstride=1, facecolors=img, shade=False)

# Show and save the static plot
plt.show()
fig.savefig("rrt_plot.jpeg", format="jpeg", dpi=300)