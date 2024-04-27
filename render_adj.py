import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from math import sin, cos, radians
from scipy.special import comb

def bernstein_poly(i, n, t):
    return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

def bezier_curve(points, num_points=100):
    n = len(points) - 1
    t = np.linspace(0, 1, num_points)
    curve = np.zeros((num_points, 3))
    for i in range(n + 1):
        curve += np.outer(bernstein_poly(i, n, t), points[i])
    return curve

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

# Get the yaw, pitch, and roll angles for each sensor
sensor_data = []
for i in range(4):
    yaw = float(input(f"Enter yaw angle for sensor {i+1} (in degrees): "))
    pitch = float(input(f"Enter pitch angle for sensor {i+1} (in degrees): "))
    roll = float(input(f"Enter roll angle for sensor {i+1} (in degrees): "))
    sensor_data.append((yaw, pitch, roll))

# Convert angles from degrees to radians
sensor_data_rad = [(radians(yaw), radians(pitch), radians(roll)) for yaw, pitch, roll in sensor_data]

# Update function for animation
def update(frame):
    # Apply translations based on the sensor data
    translated_points = np.copy(data_points)
    for i in range(len(sensor_data_rad)):
        yaw_rad, pitch_rad, roll_rad = sensor_data_rad[i]

        # Apply pitch translation
        pitch_translation = np.array([[1, 0, 0],
                                      [0, cos(pitch_rad), -sin(pitch_rad)],
                                      [0, sin(pitch_rad), cos(pitch_rad)]])
        translated_points[i] = np.dot(pitch_translation, translated_points[i])

        # Apply yaw translation
        yaw_translation = np.array([[cos(yaw_rad), -sin(yaw_rad), 0],
                                    [sin(yaw_rad), cos(yaw_rad), 0],
                                    [0, 0, 1]])
        translated_points[i] = np.dot(yaw_translation, translated_points[i])

        # Apply roll translation
        roll_translation = np.array([[cos(roll_rad), 0, sin(roll_rad)],
                                     [0, 1, 0],
                                     [-sin(roll_rad), 0, cos(roll_rad)]])
        translated_points[i] = np.dot(roll_translation, translated_points[i])

    # Update the points
    points._offsets3d = (translated_points[:, 0], translated_points[:, 1], translated_points[:, 2])

    # Update the lines
    for line in lines:
        line.remove()
    lines.clear()

    # Connect the points to form the stick figure with bending
    # Neck to upper back
    if -20 <= sensor_data[0][1] <= 0:
        control_points = np.array([translated_points[0], 
                                   translated_points[0] + np.array([0, 0, 0.5 * sensor_data[0][1]]),
                                   translated_points[1]])
        curve_points = bezier_curve(control_points)
        lines.append(ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2], 'b-')[0])
    else:
        lines.append(ax.plot(translated_points[[0, 1], 0], translated_points[[0, 1], 1], translated_points[[0, 1], 2], 'b-')[0])

    # Upper back to lower back
    if sensor_data[2][1] > sensor_data[1][1]:
        control_points = np.array([translated_points[1],
                                   translated_points[1] + np.array([0.5 * (sensor_data[2][1] - sensor_data[1][1]), 0, 0]),
                                   translated_points[2]])
        curve_points = bezier_curve(control_points)
        lines.append(ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2], 'b-')[0])
    else:
        lines.append(ax.plot(translated_points[[1, 2], 0], translated_points[[1, 2], 1], translated_points[[1, 2], 2], 'b-')[0])

    # Lower back to waist
    lines.append(ax.plot(translated_points[[2, 3], 0], translated_points[[2, 3], 1], translated_points[[2, 3], 2], 'b-')[0])

    # Scoliosis detection
    if abs(sensor_data[1][0] - sensor_data[2][0]) > 5:
        control_points = np.array([translated_points[1],
                                   translated_points[1] + np.array([0, 0.5 * (sensor_data[2][0] - sensor_data[1][0]), 0]),
                                   translated_points[2]])
        curve_points = bezier_curve(control_points)
        lines.append(ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2], 'r-')[0])

    # Add arms
    lines.append(ax.plot([translated_points[1, 0], translated_points[1, 0] - 0.5], [translated_points[1, 1], translated_points[1, 1] - 0.5], [translated_points[1, 2], translated_points[1, 2]], 'b-')[0])  # Left arm
    lines.append(ax.plot([translated_points[1, 0], translated_points[1, 0] + 0.5], [translated_points[1, 1], translated_points[1, 1] + 0.5], [translated_points[1, 2], translated_points[1, 2]], 'b-')[0])  # Right arm

    # Add legs
    lines.append(ax.plot([translated_points[3, 0], translated_points[3, 0] - 0.25], [translated_points[3, 1], translated_points[3, 1] - 0.25], [translated_points[3, 2], translated_points[3, 2] - 1], 'b-')[0])  # Left leg
    lines.append(ax.plot([translated_points[3, 0], translated_points[3, 0] + 0.25], [translated_points[3, 1], translated_points[3, 1] + 0.25], [translated_points[3, 2], translated_points[3, 2] - 1], 'b-')[0])  # Right leg

    # Add face indicator
    face_point = translated_points[0] + np.array([0, 0.5, 0])
    lines.append(ax.plot([translated_points[0, 0], face_point[0]], [translated_points[0, 1], face_point[1]], [translated_points[0, 2], face_point[2]], 'g-')[0])
    lines.append(ax.text(face_point[0], face_point[1], face_point[2], 'Face', color='g'))

    return points, *lines

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=1, interval=50, blit=True)

# Show the plot
plt.tight_layout()
plt.show()