import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import matplotlib.image as mpimg

# Add path to rrtDemo (uncommented and cleaned)
sys.path.append(os.path.join(os.path.dirname(__file__), '../rrt_demo_app/build'))
import rrtDemo

# Flags
render_gif = False
render_png = True

# View Angles
elev_angle = 10
azim_angle = -135

# Parameters
range_a_x, range_a_y = -5.0, -5.0
range_b_x, range_b_y = 5.0, 5.0
origin_x, origin_y = -5.0, 0.0
origin_time = 0.0
dest_x, dest_y = 4.5, 2.5
dest_time = 10.0
max_angle_rad = 0.3
max_dist = 2.0
min_dist = 0.01
max_interval = 5
max_time = 10.0
dim_3D = True
iteration_limit = 1000
initial_heading = 0.6
max_admissible = 7

# Tuples
range_a = (range_a_x, range_a_y, 0.0, 0.0)
range_b = (range_b_x, range_b_y, max_time, 0.0)
origin = (origin_x, origin_y, origin_time, initial_heading)
dest = (dest_x, dest_y, dest_time, 0.0)

# Occupancy map
occupancy_map = [
    ((1.0, 1.0, 12.0, 0.0), 0.5),
    ((1.0, 4.0, 12.0, 0.0), 0.5)
]
for i, y in enumerate(np.linspace(-5, 5, num=33)):
    z_centroid = 1.0 + 7 * (y + 5) / 10
    x_start, x_end = -2, 2
    z_min, z_max = 1.0, 10.0
    x = x_start + (x_end - x_start) * ((z_centroid - z_min) / (z_max - z_min))
    occupancy_map.append(((x, y, z_centroid, 0.0), 0.5))

# Initialize and build RRT
vis_rrt = rrtDemo.RRT(
    occupancy_map, range_a, range_b, origin, dest,
    max_angle_rad, max_dist, min_dist, max_interval,
    max_time, dim_3D, iteration_limit, max_admissible
)

while not vis_rrt.isComplete():
    vis_rrt.stepRRT()

print("RRT build complete:", vis_rrt.isComplete())
print("Number of nodes:", vis_rrt.getNodeCount() - 2)

# Collect node data
node_count = vis_rrt.getNodeCount()
all_node_xs, all_node_ys, all_node_zs = [], [], []
fwd_dict = {}
for i in range(node_count):
    node = vis_rrt.getNodeAt(i)
    if node:
        all_node_xs.append(node.xCrdnt())
        all_node_ys.append(node.yCrdnt())
        all_node_zs.append(node.time())
        fwd_indices = vis_rrt.getForwardIndices(i)
        fwd_dict[i] = list(fwd_indices)

# First node
first_node = vis_rrt.getNodeAt(0)
first_x, first_y, first_z = (first_node.xCrdnt(), first_node.yCrdnt(), first_node.time()) if first_node else (0, 0, 0)

# Animation params
num_frames = len(all_node_xs)
duration_ms = 15000
interval_ms = max(1, duration_ms // num_frames)

# Green indices (destination nodes)
xy_tol = 1e-2
green_indices = [i for i in range(node_count) if vis_rrt.getNodeAt(i) and
                 abs(vis_rrt.getNodeAt(i).xCrdnt() - dest_x) <= xy_tol and
                 abs(vis_rrt.getNodeAt(i).yCrdnt() - dest_y) <= xy_tol]

# Image preparation (if enabled)
def prepare_images():
    # Blue man
    img = mpimg.imread("blue_man.png")
    if img.dtype == np.uint8:
        img = img.astype(np.float32) / 255.0
    img = np.flipud(img)
    img = np.fliplr(img)
    img_height = 4.0
    img_width = img.shape[1] / img.shape[0] * img_height
    fixed_x = -2.2
    img_y = np.linspace(-6, -6 + img_width, img.shape[1])
    img_z = np.linspace(0, img_height, img.shape[0])
    img_y, img_z = np.meshgrid(img_y, img_z)
    img_x = np.full_like(img_y, fixed_x)
    
    # Yield sign
    yield_img = mpimg.imread("yield.png")
    if yield_img.dtype == np.uint8:
        yield_img = yield_img.astype(np.float32) / 255.0
    yield_img = np.flipud(yield_img)
    yield_img = np.fliplr(yield_img)
    yield_height = 3.0
    yield_width = yield_img.shape[1] / yield_img.shape[0] * yield_height
    yield_positions = [(-3.5, 0, 12.0), (-3.5, 3, 12.0)]
    
    return (img_x, img_y, img_z, img), (yield_img, yield_width, yield_height, yield_positions)

image_data = prepare_images() if render_png else None

# Plot obstacles consistently
def plot_obstacles(ax, occupancy_map):
    w = 0.5  # Fixed width/height
    for occ in occupancy_map:
        (x, y, z, _), _ = occ
        if (x, y, z) == (1.0, 1.0, 12.0) or (x, y, z) == (1.0, 4.0, 12.0):
            # Special vertical bars: shifted, full height from 0, red
            ax.bar3d(x - w / 2 - 3.5, y - w / 2 - 2, 0, w, w, z, shade=True, color='red', alpha=0.8)
        else:
            # Sloped bars: no shift, height 1 centered at z, cyan
            h = 1.0
            ax.bar3d(x - w / 2, y - w / 2, z - h / 2, w, w, h, shade=True, color='cyan', alpha=0.8)

# Plot tree elements consistently
def plot_tree(ax, all_node_xs, all_node_ys, all_node_zs, fwd_dict, green_indices, frame=None, dest_x=None, dest_y=None, dest_time=None, first_x=None, first_y=None, first_z=None):
    # Blue dot at first node
    if first_x is not None:
        ax.scatter(first_x, first_y, first_z, color='blue', s=50)
    
    # Green line at destination
    if dest_x is not None:
        ax.plot([dest_x, dest_x], [dest_y, dest_y], [0, dest_time], color='green', linewidth=3)
    
    # Nodes up to frame (or all)
    end_frame = frame + 1 if frame is not None else len(all_node_xs)
    current_xs = all_node_xs[:end_frame]
    current_ys = all_node_ys[:end_frame]
    current_zs = all_node_zs[:end_frame]
    if current_xs:
        ax.scatter(current_xs, current_ys, current_zs, color='orange', s=20)
    
    # Connections up to frame (or all)
    end_range = range(end_frame)
    for i in end_range:
        fwd_list = fwd_dict.get(i, [])
        x1, y1, z1 = all_node_xs[i], all_node_ys[i], all_node_zs[i]
        for fwd_idx in fwd_list:
            if (frame is None or fwd_idx <= frame) and fwd_idx < len(all_node_xs):
                x2, y2, z2 = all_node_xs[fwd_idx], all_node_ys[fwd_idx], all_node_zs[fwd_idx]
                ax.plot([x1, x2], [y1, y2], [z1, z2], color='blue', linewidth=1, alpha=0.7)
    
    # Green dots up to frame (or all)
    for idx in green_indices:
        if frame is None or idx <= frame:
            ax.scatter(all_node_xs[idx], all_node_ys[idx], all_node_zs[idx], color='green', s=60)

# Plot images if enabled
def plot_images(ax, image_data):
    if not render_png or image_data is None:
        return
    (img_x, img_y, img_z, img), (yield_img, yield_width, yield_height, yield_positions) = image_data
    
    # Blue man
    ax.plot_surface(img_x, img_y, img_z, rstride=1, cstride=1, facecolors=img, shade=False)
    
    # Yield signs
    w = 0.5
    for x, y, _ in yield_positions:
        fixed_x = x  # Consistent with static
        fixed_y = y - w / 2 - 2
        img_y_loc = np.linspace(fixed_y, fixed_y + yield_width, yield_img.shape[1])
        img_z_loc = np.linspace(0, yield_height, yield_img.shape[0])
        img_y_loc, img_z_loc = np.meshgrid(img_y_loc, img_z_loc)
        img_x_loc = np.full_like(img_y_loc, fixed_x)
        ax.plot_surface(img_x_loc, img_y_loc, img_z_loc, rstride=1, cstride=1, facecolors=yield_img, shade=False)

# Set common axis properties
def set_axis_props(ax):
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Time')
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_zlim(0, 12)
    ax.view_init(elev=elev_angle, azim=azim_angle)  # Consistent view

# Static plot
fig = plt.figure(figsize=(9.6, 7.2))
ax = fig.add_subplot(111, projection='3d')

plot_obstacles(ax, occupancy_map)
plot_tree(ax, all_node_xs, all_node_ys, all_node_zs, fwd_dict, green_indices,
          frame=None, dest_x=dest_x, dest_y=dest_y, dest_time=dest_time,
          first_x=first_x, first_y=first_y, first_z=first_z)
plot_images(ax, image_data)

set_axis_props(ax)
ax.set_title('3D Visualization of RRT Node Placement')

plt.show()
fig.savefig("rrt_plot.jpeg", format="jpeg", dpi=300)
print("Static plot saved as 'rrt_plot.jpeg'.")

if render_gif:
    fig_anim = plt.figure(figsize=(12.8, 9.6))
    ax_anim = fig_anim.add_subplot(111, projection='3d')
    
    def animate(frame):
        ax_anim.cla()
        plot_obstacles(ax_anim, occupancy_map)
        plot_tree(ax_anim, all_node_xs, all_node_ys, all_node_zs, fwd_dict, green_indices,
                  frame=frame, dest_x=dest_x, dest_y=dest_y, dest_time=dest_time,
                  first_x=first_x, first_y=first_y, first_z=first_z)
        plot_images(ax_anim, image_data)
        set_axis_props(ax_anim)
        ax_anim.set_title(f'3D Animation of RRT Node Placement (Frame {frame + 1}/{num_frames})')
    
    anim = animation.FuncAnimation(fig_anim, animate, frames=num_frames, interval=interval_ms, blit=False, repeat=True)
    anim.save('rrt_animation.gif', writer='pillow')
    print("GIF saved as 'rrt_animation.gif'.")
    plt.close(fig_anim)