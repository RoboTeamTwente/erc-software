import numpy as np

u = np.array((-3, 5, -6))
v = np.array((7, -4, 2))

d = np.linalg.norm(u - v)
print(d)
