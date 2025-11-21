import numpy as np

# NOTE: Theta = smallest nonnegative angle between two non-zero vectors u and v
# NOTE: cos Theta = (u*v)/(||u|| * ||v||)

u = np.array([5, -3, -1])
v = np.array([2, 4, -5])

cos_theta = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
print("cos θ =", cos_theta)

theta = np.arccos(cos_theta)  # angle in radians
print("θ (radians) =", theta)

theta_deg = np.degrees(theta)  # angle in degrees
print("θ (degrees) =", theta_deg)
