import os
import cv2
import numpy as np
from glob import glob
import random
from tqdm import tqdm

# Define the augmentation functions
def rotate_image(image, angle):
    """Rotate the image by a specific angle."""
    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    return cv2.warpAffine(image, matrix, (w, h))

def scale_image(image, scale):
    """Scale the image."""
    (h, w) = image.shape[:2]
    return cv2.resize(image, (int(w * scale), int(h * scale)))

def translate_image(image, tx, ty):
    """Translate the image."""
    matrix = np.float32([[1, 0, tx], [0, 1, ty]])
    return cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]))

def shear_image(image, shear):
    """Shear the image."""
    matrix = np.float32([[1, shear, 0], [0, 1, 0]])
    return cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]))

def adjust_brightness(image, factor):
    """Adjust brightness."""
    return cv2.convertScaleAbs(image, alpha=factor, beta=0)

def adjust_saturation(image, factor):
    """Adjust saturation."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv[:, :, 1] = cv2.multiply(hsv[:, :, 1], factor)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

def adjust_hue(image, factor):
    """Adjust hue while ensuring it stays within valid bounds."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Convert to a larger type (int16) to prevent overflow
    hsv = hsv.astype(np.int16)
    
    # Apply hue adjustment and wrap around within [0, 179]
    hsv[:, :, 0] = (hsv[:, :, 0] + factor) % 180  # Wrap hue values around 0-179
    hsv[:, :, 0] = np.clip(hsv[:, :, 0], 0, 179)  # Ensure hue stays in [0, 179]
    
    # Convert back to uint8
    hsv = hsv.astype(np.uint8)
    
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def grayscale_conversion(image):
    """Convert image to grayscale."""
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def add_salt_and_pepper_noise(image, prob=0.02):
    """Add salt-and-pepper noise."""
    output = np.copy(image)
    black = np.zeros_like(image)
    white = 255 * np.ones_like(image)
    mask = np.random.rand(*image.shape[:2]) < prob
    output[mask] = white[mask]
    mask = np.random.rand(*image.shape[:2]) < prob
    output[mask] = black[mask]
    return output

# Main function for augmentation
def augment_dataset(input_folder, output_folder, augment_count=5):
    """Augment dataset and save augmented images."""
    os.makedirs(output_folder, exist_ok=True)
    image_paths = glob(os.path.join(input_folder, '*'))
    
    for image_path in tqdm(image_paths, desc="Augmenting Images"):
        image = cv2.imread(image_path)
        filename = os.path.basename(image_path)
        name, ext = os.path.splitext(filename)
        
        # Apply augmentations
        for i in range(augment_count):
            aug_image = image.copy()
            
            # Randomly apply augmentations
            if random.random() > 0.5:
                aug_image = rotate_image(aug_image, random.randint(-30, 30))
            if random.random() > 0.5:
                aug_image = scale_image(aug_image, random.uniform(0.8, 1.2))
            if random.random() > 0.5:
                aug_image = translate_image(aug_image, random.randint(-20, 20), random.randint(-20, 20))
            if random.random() > 0.5:
                aug_image = shear_image(aug_image, random.uniform(-0.2, 0.2))
            if random.random() > 0.5:
                aug_image = adjust_brightness(aug_image, random.uniform(0.7, 1.3))
            if random.random() > 0.5:
                aug_image = adjust_saturation(aug_image, random.uniform(0.7, 1.3))
            if random.random() > 0.5:
                aug_image = adjust_hue(aug_image, random.randint(-10, 10))
            if random.random() > 0.5:
                aug_image = grayscale_conversion(aug_image)
            if random.random() > 0.5:
                aug_image = add_salt_and_pepper_noise(aug_image)

            # Save augmented image
            output_path = os.path.join(output_folder, f"{name}_aug_{i}{ext}")
            cv2.imwrite(output_path, aug_image)

# Input and output folder paths
input_folder = "img"
output_folder = "aug"

# Augment the dataset
augment_dataset(input_folder, output_folder, augment_count=5)
