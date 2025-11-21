import numpy as np
import math

# Dot Product
# NOTE: if u and v are both 2x1 vectors with (x_u,y_u) respectively, then
# NOTE: Dot product is: u*v = x_u*x_v + y_u*y_v

u = np.array([4, 3])
v = np.array([2, 8])

# print("Dot Product Expected: 32")
# print("Dot Product Actual:", np.dot(u, v))

# The magnitude of a vector


# function definition to compute magnitude o f the vector
def magnitude(vector):
    return math.sqrt(sum(pow(element, 2) for element in vector))


# displaying the original vector
# print("Vector:", v)

# computing and displaying the magnitude of the vector
# print("Magnitude of the Vector:", magnitude(v))
# print("Magnitude squared of the Vector: ", pow(magnitude(v), 2))


# Projection of U onto V


def get_vector_proj(u, v):
    dot_prod = np.dot(u, v)
    magnitude_sq = pow(magnitude(v), 2)
    vector_proj = dot_prod / magnitude_sq * v
    return vector_proj


# NOTE: A more concise version is basically this
def proj(u, v):
    return np.dot(u, v) / np.dot(v, v) * v


# print("The projection of U onto V is: ", get_vector_proj(u, v))
# NOTE: You can change the vectors u and v to get whatever proj you want. u is the vector projected onto v
u = np.array([-2, 5])
v = np.array([6, -5])
print("The projection of U onto V is: ", proj(u, v))
