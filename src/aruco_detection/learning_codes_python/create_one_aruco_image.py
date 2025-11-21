import cv2
import cv2.aruco as aruco
import os

# Aruco: Make dictionary, set ID and set size
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
marker_id = 25
marker_size = 200

# Set saving the image in a specific folder
output_dir = "aruco-detection/images"
output_file = f"marker{marker_id}.png"

os.makedirs(output_dir, exist_ok=True)

# Generate the Aruco Tag Itself
marker_image = aruco.generateImageMarker(dictionary, marker_id, marker_size)

# Write to the folder
save_path = os.path.join(output_dir, output_file)
cv2.imwrite(save_path, marker_image)

# Show the Aruco tag in a nice window as well
cv2.imshow(f"ArUco Marker {marker_id}", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
