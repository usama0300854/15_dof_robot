import os
import pyheif
from PIL import Image

def heic_to_jpg(input_path, output_path):
    """
    Converts a HEIC file to JPG format.
    
    :param input_path: Path to the input HEIC file.
    :param output_path: Path to save the converted JPG file.
    """
    try:
        # Read HEIC file
        heif_file = pyheif.read(input_path)
        
        # Convert to a PIL Image
        image = Image.frombytes(
            heif_file.mode, 
            heif_file.size, 
            heif_file.data,
            "raw",
            heif_file.mode,
            heif_file.stride,
        )
        
        # Save as JPG
        image.save(output_path, "JPEG")
        print(f"Converted: {input_path} → {output_path}")
    except Exception as e:
        print(f"Failed to convert {input_path}: {e}")

# Example usage
if __name__ == "__main__":
    input_directory = "heic"
    output_directory = "jpg"
    
    # Create output directory if it doesn't exist
    os.makedirs(output_directory, exist_ok=True)

    # Convert all HEIC files in the directory
    for filename in os.listdir(input_directory):
        if filename.lower().endswith(".heic"):
            input_file = os.path.join(input_directory, filename)
            output_file = os.path.join(output_directory, f"{os.path.splitext(filename)[0]}.jpg")
            heic_to_jpg(input_file, output_file)
