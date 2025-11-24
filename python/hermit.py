import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicHermiteSpline

# Define the 5 data points and their derivatives
x = np.array([0, 1, 2, 3, 4])  # x-coordinates
y = np.array([0, 1, 0, -1, 0])  # y-coordinates
dydx = np.array([1, 0, -1, 0, 1])  # derivatives

# Create the Hermite interpolator
hermite = CubicHermiteSpline(x, y, dydx)

# Generate interpolated points for smooth plotting
x_interp = np.linspace(x.min(), x.max(), 1000)
y_interp = hermite(x_interp)

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(x_interp, y_interp, 'b-', label='Hermite Interpolation')
plt.plot(x, y, 'ro', label='Data Points', markersize=8)

# Add derivative indicators
for xi, yi, dyi in zip(x, y, dydx):
    plt.arrow(xi, yi, 0.2, 0.2*dyi, head_width=0.05, 
              head_length=0.1, fc='k', ec='k')

plt.xlabel('x')
plt.ylabel('y')
plt.title('Hermite Interpolation of 5 Points')
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.show()