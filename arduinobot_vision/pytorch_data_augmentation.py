import torch
from torchvision import transforms
from PIL import Image
import os
import random
import numpy as np
import matplotlib.pyplot as plt

# Input folder containing images
input_folder = 'path_to_your_images_folder'  # Replace with your images folder path

# Output folder for augmented images
output_folder = 'augmented_images_pytorch'

# Ensure the output folder exists
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Define the transformations (augmentation techniques)
transformations = [
    # Rotation
    transforms.Compose([transforms.RandomRotation(degrees=30)]),
    
    # Scaling / Zooming
    transforms.Compose([transforms.RandomResizedCrop(224, scale=(0.8, 1.2))]),

    # Translation (Shifting)
    transforms.Compose([transforms.RandomAffine(0, translate=(0.1, 0.1))]),

    # Shearing
    transforms.Compose([transforms.RandomAffine(0, shear=10)]),

    # Brightness Adjustment
    transforms.Compose([transforms.ColorJitter(brightness=0.2)]),

    # Saturation Adjustment
    transforms.Compose([transforms.ColorJitter(saturation=0.2)]),

    # Hue Adjustment
    transforms.Compose([transforms.ColorJitter(hue=0.2)]),

    # Grayscale Conversion
    transforms.Compose([transforms.Grayscale(num_output_channels=3)]),  # Grayscale but keep 3 channels

    # Gaussian Noise (manually added, not part of torchvision.transforms)
    lambda img: add_gaussian_noise(img, mean=0, std=25),

    # Salt-and-Pepper Noise (manually added, not part of torchvision.transforms)
    lambda img: add_salt_and_pepper_noise(img, prob=0.02)
]

# Function to add Gaussian Noise to an image
def add_gaussian_noise(image, mean=0, std=25):
    np_image = np.array(image).astype(np.float32)
    noise = np.random.normal(mean, std, np_image.shape)
    noisy_image = np.clip(np_image + noise, 0, 255)
    return Image.fromarray(noisy_image.astype(np.uint8))

# Function to add Salt-and-Pepper Noise to an image
def add_salt_and_pepper_noise(image, prob=0.02):
    np_image = np.array(image)
    total_pixels = np_image.size
    num_salt = int(total_pixels * prob)
    num_pepper = int(total_pixels * prob)

    # Add salt noise
    salt_coords = [np.random.randint(0, i - 1, num_salt) for i in np_image.shape]
    np_image[salt_coords[0], salt_coords[1], :] = 255

    # Add pepper noise
    pepper_coords = [np.random.randint(0, i - 1, num_pepper) for i in np_image.shape]
    np_image[pepper_coords[0], pepper_coords[1], :] = 0

    return Image.fromarray(np_image)

# Apply each transformation and save the augmented images
def augment_and_save(image, output_folder, augment_count=5, base_filename='aug_image'):
    for i, transform in enumerate(transformations):
        for j in range(augment_count):
            # Apply the transformation
            if callable(transform):
                aug_image = transform(image)
            else:
                aug_image = transform(image)
            
            # Save the augmented image
            output_path = os.path.join(output_folder, f'{base_filename}_{i}_aug_{j}.jpg')
            aug_image.save(output_path)

            # Optionally display the augmented image (for visual inspection)
            # Comment out if not needed
            plt.subplot(1, augment_count, j+1)
            plt.imshow(aug_image)
            plt.axis('off')
    
    # Show the images after augmentation
    plt.show()

# Process all images in the input folder
def process_images_in_folder(input_folder, output_folder, augment_count=5):
    # Loop through each image in the input folder
    for filename in os.listdir(input_folder):
        file_path = os.path.join(input_folder, filename)

        if os.path.isfile(file_path) and filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            image = Image.open(file_path)

            # Generate a base filename (without extension) for the augmented images
            base_filename = os.path.splitext(filename)[0]

            print(f'Augmenting {filename}...')
            augment_and_save(image, output_folder, augment_count=augment_count, base_filename=base_filename)

# Run the augmentation process on all images in the folder
process_images_in_folder(input_folder, output_folder, augment_count=5)
