import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

# Data from the image and additional coordinates
test_numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9]
x_coords = [1426, 2645, 3880, 1426, 1426, 2051, 2051, 2203, 2645]
y_coords = [0, 0, 0, -152, -305, 305, 610, 762, 457]

# Create the plot
plt.figure(figsize=(12, 8))

# Plot test numbers at specified coordinates
for test_num, x, y in zip(test_numbers, x_coords, y_coords):
    plt.text(x, y, str(test_num), fontsize=12, ha='center', va='center')

# Plot thick black line at origin from Y=177mm to Y=-177mm
plt.plot([0, 0], [177, -177], color='black', linewidth=3)

# Add microphone markers at the ends of the line
plt.scatter([0, 0], [177, -177], s=200, color='blue')

# Set axis limits and labels
plt.xlim(-100, max(x_coords) * 1.1)
plt.ylim(max(max(y_coords) + 100, 200), min(min(y_coords) - 100, -200))
plt.xlabel('X-position (mm)')
plt.ylabel('Y-position (mm)')

# Set title
plt.title('Transmitter Positions for Each Test')

# Add grid
plt.grid(True, linestyle='--', alpha=0.7)

# Show the plot
plt.show()