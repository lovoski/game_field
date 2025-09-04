import numpy as np
import matplotlib.pyplot as plt

num = 25

deg = np.random.random(num)*np.pi*2
length = np.clip(np.random.randn(num), -1, 1)
# length /= max(abs(np.max(length)), abs(np.min(length)))
x = length*np.cos(deg)
y = length*np.sin(deg)

coord = np.stack([x,y]).transpose()
for i in range(num):
  print(f'vec2({x[i]},{y[i]}),')
# print(coord)

plt.figure()
plt.scatter(x, y)
plt.show()
