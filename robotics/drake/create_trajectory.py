import numpy as np
from scipy.spatial.transform import Rotation as R

def generate_translations(points: np.ndarray, start_point=np.array([0, 0, 0])):
    translations = []
    previous_point = start_point
    for point in points:
        # Calculate translation vector from the previous point to the current point
        translation_vector = point - previous_point
        # Store the translation vector
        translations.append(translation_vector)
        # Update the previous point for the next iteration
        previous_point = point

    return translations


def generate_rotation_and_translations(points: np.ndarray, start_point=np.array([0, 0, 0])):
    rotation_and_translations = []
    previous_point = start_point

    for point in points:
        # Calculate translation vector from the previous point to the current point
        translation_vector = point - previous_point

        # If the translation vector is not zero, calculate the direction vector
        if np.linalg.norm(translation_vector) != 0:
            direction_vector = translation_vector / np.linalg.norm(translation_vector)
        else:
            # If the translation vector is zero, keep the direction vector from the previous iteration
            # or default to a unit vector along the z-axis if this is the first iteration
            direction_vector = np.array([0, 0, 1])

        # Assume the up vector to be [0, 0, 1]
        up_vector = np.array([0, 0, 1])

        # Cross product to find the right vector
        right_vector = np.cross(up_vector, direction_vector)
        right_vector /= np.linalg.norm(right_vector)

        # Re-calculate the up vector to ensure orthogonality
        up_vector = np.cross(direction_vector, right_vector)

        # Form the rotation matrix
        rotation_matrix = np.vstack([right_vector, up_vector, direction_vector])

        # Convert the rotation matrix to a quaternion
        rotation = R.from_matrix(rotation_matrix)

        if not np.isnan(rotation.as_matrix()).any():
            # Store the quaternion and translation vector
            rotation_and_translations.append((rotation.as_matrix(), translation_vector))

        # Update the previous point for the next iteration
        previous_point = point

    return rotation_and_translations



if __name__ == "__main__":
    # Example usage:
    points = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    qt_pairs = generate_rotation_and_translations(points)
    for qt_pair in qt_pairs:
        quaternion, translation = qt_pair
        print(f'Quaternion: {quaternion}, Translation: {translation}')

    # Lets test with circles.
    points = np.array([[np.sin(theta), np.cos(theta), 0] for theta in np.arange(0, 2*np.pi, 0.1)])

    qt_pairs = generate_rotation_and_translations(points)
    for qt_pair in qt_pairs:
        quaternion, translation = qt_pair
        print(f'Quaternion: {quaternion}, Translation: {translation}')


