import numpy as np
import math

u = np.array([5, 2, 4])
v = np.array([3, 4, -7])
v1 = np.array([4, 2, 6])

# dot product between two vectors
print(np.dot(u, v))

# length of a 3D vector: it's literally the dot product with itself
print(math.sqrt(np.dot(v1, v1)))
print(np.dot(v, v))
