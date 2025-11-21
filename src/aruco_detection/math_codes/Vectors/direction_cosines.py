import numpy as np

u = np.array([4, 5, 2])

# norm is the magnitude of the vector u
# ||u|| = sqrt(u-x^2 + u_y^2 + u_z^2)
norm = np.linalg.norm(u)

# cos_alpha = u_x/||u||;
# cos_beta = u_y/||u||;
# cos_theta = u_z[2]/||u||;
cos_alpha = u[0] / norm  # with x-axis
cos_beta = u[1] / norm  # with y-axis
cos_theta = u[2] / norm  # with z-axis

# print(norm)
print(
    f"""The three cosins alpha, beta and theta (x,y,z) are:
    cos_alpha = {cos_alpha},
    cos_beta = {cos_beta},
    cos_theta = {cos_theta}"""
)
