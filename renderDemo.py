import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Initialize the data points
data_points = np.array([[0, 0, 0],  # Neck
                        [0, 0, -1],  # Upper back
                        [0, 0, -2],  # Lower back
                        [0, 0, -3]])  # Waist

# Create the figure and subplots
fig = plt.figure(figsize=(10, 5))
ax = fig.add_subplot(111, projection='3d')

# Set the limits and labels for the axes
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-4, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Stick Figure')

# Initialize the lines and points
lines = []
points = ax.scatter([], [], [], c='r', marker='o')

# Update function for animation
def update(frame):
    # Update the points
    points._offsets3d = (data_points[:, 0], data_points[:, 1], data_points[:, 2])

    # Update the lines
    for line in lines:
        line.remove()
    lines.clear()

    # Connect the points to form the stick figure
    lines.append(ax.plot(data_points[[0, 1], 0], data_points[[0, 1], 1], data_points[[0, 1], 2], 'b-')[0])  # Neck to upper back
    lines.append(ax.plot(data_points[[1, 2], 0], data_points[[1, 2], 1], data_points[[1, 2], 2], 'b-')[0])  # Upper back to lower back
    lines.append(ax.plot(data_points[[2, 3], 0], data_points[[2, 3], 1], data_points[[2, 3], 2], 'b-')[0])  # Lower back to waist

    # Add arms
    lines.append(ax.plot([data_points[1, 0], data_points[1, 0] - 0.5], [data_points[1, 1], data_points[1, 1] - 0.5], [data_points[1, 2], data_points[1, 2]], 'b-')[0])  # Left arm
    lines.append(ax.plot([data_points[1, 0], data_points[1, 0] + 0.5], [data_points[1, 1], data_points[1, 1] + 0.5], [data_points[1, 2], data_points[1, 2]], 'b-')[0])  # Right arm

    # Add legs
    lines.append(ax.plot([data_points[3, 0], data_points[3, 0] - 0.25], [data_points[3, 1], data_points[3, 1] - 0.25], [data_points[3, 2], data_points[3, 2] - 1], 'b-')[0])  # Left leg
    lines.append(ax.plot([data_points[3, 0], data_points[3, 0] + 0.25], [data_points[3, 1], data_points[3, 1] + 0.25], [data_points[3, 2], data_points[3, 2] - 1], 'b-')[0])  # Right leg

    return points, *lines

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=1, interval=50, blit=True)

# Show the plot
plt.tight_layout()
plt.show()