import numpy as np

# NOTE: The unit vector v_hat (called 'v hat') is obtained from a base vector v of any size.
# NOTE: Formula: v_hat = v/||v||  -> Initial vector divided by its magnitude


def unit_vect(v):
    return v / np.linalg.norm(v)


v = np.array([-8, 12])
v_unit = unit_vect(v)

print(f"""The unit vector of {v} is:
      {v_unit}""")
