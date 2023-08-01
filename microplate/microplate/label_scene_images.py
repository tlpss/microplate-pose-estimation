import pathlib
import os 

if __name__ == "__main__":
    directory = pathlib.Path(__file__).parent / "scene_images"

    images_to_label_indices = [0,18,34]

    images = os.listdir(directory)
    images = sorted(images)
    images_to_label = [images[i] for i in images_to_label_indices]

    for image in images_to_label:
        # open image in opencv window and annotate the keypoint configuration.
        # save the keypoints in a json file.
        pass 