import cv2
import numpy as np

def generate_map(clearance=25, map_height=3000, map_width=5400):
    mm_to_pixels = 0.5
    width_pixels = int(map_width * mm_to_pixels)
    height_pixels = int(map_height * mm_to_pixels)

    x_mm = np.arange(width_pixels) / mm_to_pixels
    y_mm = (height_pixels - np.arange(height_pixels)) / mm_to_pixels
    X_mm, Y_mm = np.meshgrid(x_mm, y_mm)

    # Load obstacle image and convert to binary mask
    img_path = "maze.png"
    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(f"Could not load image at {img_path}")
    
    # Resize image to match our map dimensions if needed
    if img.shape[0] != height_pixels or img.shape[1] != width_pixels:
        img = cv2.resize(img, (width_pixels, height_pixels))
    
    # Convert to grayscale and threshold to create binary mask
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, obstacle_mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    obstacle_mask = obstacle_mask == 0  # Assuming black represents obstacles

    edge_clearance = int(1)
    edge_mask = (
        (X_mm <= edge_clearance) | 
        (X_mm >= map_width - edge_clearance) | 
        (Y_mm <= edge_clearance) | 
        (Y_mm >= map_height - edge_clearance)
    )

    image = np.ones((height_pixels, width_pixels, 3), dtype=np.uint8) * 255
    image[obstacle_mask] = [0, 0, 0]
    image[edge_mask] = [0, 0, 255]

    clearance_pixels = int(clearance/mm_to_pixels)  
    kernel = np.ones((2*clearance_pixels+1, 2*clearance_pixels+1), np.uint8)
    
    dilated = cv2.dilate((obstacle_mask*255).astype(np.uint8), kernel, iterations=1)
    obstacle_clearance_area = (dilated > 0) & ~obstacle_mask
    image[obstacle_clearance_area] = [0, 0, 255]
    
    total_obstacle_area = obstacle_mask | obstacle_clearance_area | edge_mask

    planning_image = np.ones((height_pixels, width_pixels, 3), dtype=np.uint8) * 255
    planning_image[total_obstacle_area] = 0

    return image

map_img = generate_map()
map_img = cv2.resize(map_img, (640, 480))
cv2.imshow("Map", map_img)
cv2.waitKey(0)
cv2.destroyAllWindows()