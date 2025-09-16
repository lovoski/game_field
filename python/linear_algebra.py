import numpy as np

A = np.array([
  [0.5, 0],
  [0, 0.5]
])

U, S, V = np.linalg.svd(A, compute_uv=True)

print(U)
print(S)
print(V)
