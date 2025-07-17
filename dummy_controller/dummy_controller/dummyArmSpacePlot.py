import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# set the range
x_min, x_max = -0.25, 0.25
y_min, y_max = -0.5, -0.0
z_min, z_max = 0.0, 0.5

# collect the scatter point
xs, ys, zs, colors = [], [], [], []

with open('result.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        x = float(row['x'])
        y = float(row['y'])
        z = float(row['z'])
        valid = row['valid'].lower() == 'true'
        color = 'green' if valid else 'red'

        if not (valid and x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
            continue  # just display the points in range
       
        xs.append(x)
        ys.append(y)
        zs.append(z)
        colors.append(color)

# display
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xs, ys, zs, c=colors, marker='o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Filtered Position Validity Map')

ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_zlim(z_min, z_max)

plt.show()

