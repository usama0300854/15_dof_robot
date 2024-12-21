import json
import os
import cv2

def convert_to_yolo(json_file, img_folder, yolo_output_folder):
    with open(json_file) as f:
        data = json.load(f)
    
    image_path = os.path.join(img_folder, data['imagePath'])
    image = cv2.imread(image_path)
    img_height, img_width = image.shape[:2]

    yolo_data = []
    for shape in data['shapes']:
        label = shape['label']
        points = shape['points']
        
        # Assuming rectangular bounding boxes (using top-left and bottom-right points)
        x_min = min(points[0][0], points[1][0])
        y_min = min(points[0][1], points[1][1])
        x_max = max(points[0][0], points[1][0])
        y_max = max(points[0][1], points[1][1])
        
        # Calculate YOLO format
        x_center = ((x_min + x_max) / 2) / img_width
        y_center = ((y_min + y_max) / 2) / img_height
        width = (x_max - x_min) / img_width
        height = (y_max - y_min) / img_height
        
        # Class ID (change according to your classes)
        class_id = 0  # Replace this with your actual class ID if you have multiple classes
        yolo_data.append(f"{class_id} {x_center} {y_center} {width} {height}")
    
    # Write YOLO annotation file
    output_path = os.path.join(yolo_output_folder, os.path.splitext(os.path.basename(json_file))[0] + ".txt")
    with open(output_path, "w") as f:
        f.write("\n".join(yolo_data))

# Usage example
json_file = "path/to/annotation.json"
img_folder = "path/to/images"
yolo_output_folder = "path/to/yolo_annotations"
convert_to_yolo(json_file, img_folder, yolo_output_folder)
