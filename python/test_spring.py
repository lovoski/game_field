from scipy.signal import savgol_filter
import numpy as np
import matplotlib.pyplot as plt

t = np.arange(1000) * 0.01
y_first = np.sin(t[:500])
y_second = np.sin(t[500:]-1) + 0.02*np.random.randn(500)
wnd_length = 500

y_second_blend = y_second.copy()
decay_rate = np.log(1024*1024)/(wnd_length*np.log(np.e))
for i in range(wnd_length):
  x = y_first[-1]-y_second[i]
  v = y_first[-1]-y_first[-2]
  x_prev = x
  x = (x_prev + (v + decay_rate * x_prev) * i) * np.exp(-decay_rate * i)
  v = (v + decay_rate * x_prev) * np.exp(-decay_rate * i) - decay_rate * x
  y_second_blend[i] += x

decay_rate = np.log(1024*1024)/(wnd_length*np.log(np.e))
y_second_filtered = savgol_filter(y_second, 70, 2)
y_second_filtered_blend = y_second_filtered.copy()
for i in range(wnd_length):
  x = y_first[-1]-y_second_filtered[i]
  v = y_first[-1]-y_first[-2]
  x_prev = x
  x = (x_prev + (v + decay_rate * x_prev) * i) * np.exp(-decay_rate * i)
  v = (v + decay_rate * x_prev) * np.exp(-decay_rate * i) - decay_rate * x
  y_second_filtered_blend[i] += x

plt.figure()
plt.plot(t[:500], y_first, label='y_first')
plt.plot(t[500:], y_second, label='y_second')
# plt.plot(t[500:], y_second_blend, label='y_second_blend')
# plt.plot(t[500:], y_second_filtered, label='y_second_filtered')
plt.plot(t[500:], y_second_filtered_blend, label='y_second_filtered_blend')
plt.legend()
plt.show()