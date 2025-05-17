#!/usr/bin/env python3

import cv2
import numpy as np
import math
import random
import os
from ament_index_python.packages import get_package_share_directory
# from matplotlib import pyplot as plt
# from matplotlib import animation
from tf_transformations import euler_from_quaternion

# === Maze loading and scaling ===
# image_path = '/home/adil/project_5_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/scripts/maze.png'
package_share = get_package_share_directory('turtlebot3_gazebo')
image_path = os.path.join(package_share, 'scripts', 'maze.png')
maze = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
if maze is None:
    raise FileNotFoundError(f"Could not load image at path: {image_path}")

# Scale to 1100x600 while preserving hard edges
maze = cv2.resize(maze, (1600, 800), interpolation=cv2.INTER_NEAREST)
_, binary_maze = cv2.threshold(maze, 127, 255, cv2.THRESH_BINARY)
binary_maze = cv2.bitwise_not(binary_maze)
# plt.imshow(binary_maze)
# plt.show()
kernel = np.ones((30, 30), np.uint8)

# Apply dilation
binary_maze = cv2.dilate(binary_maze, kernel, iterations=2)
binary_maze = cv2.bitwise_not(binary_maze)
# plt.imshow(binary_maze)
# plt.show()

occupancy_grid = (binary_maze == 255).astype(np.uint8)
height, width = occupancy_grid.shape

# Coordinate system conversion
def invert_y(y, height): 
    return height - 1 - y

class Node:
    def __init__(self, x, y, yaw=None):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0
        self.yaw = yaw  # Store yaw angle here

def distance(n1, n2): 
    return math.hypot(n1.x - n2.x, n1.y - n2.y)

def sample_point(width, height, goal, goal_bias=0.3):
    return goal if random.random() < goal_bias else (
        random.randint(0, width - 1), 
        random.randint(0, height - 1)
    )

def is_collision_free(p1, p2, grid):
    x1, y1 = p1
    x2, y2 = p2
    steps = max(abs(x2 - x1), abs(y2 - y1)) + 1  # Dynamic step count
    x_points = np.linspace(x1, x2, steps).astype(int)
    y_points = np.linspace(y1, y2, steps).astype(int)
    
    for x, y in zip(x_points, y_points):
        if not (0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]):
            return False
        if grid[invert_y(y, grid.shape[0]), x] == 0:
            return False
    return True

def rrt_star(start, goal, grid, max_iter=5000, base_radius=30, goal_bias=0.3):
    height, width = grid.shape
    start_node = Node(*start)
    goal_node = Node(*goal)
    nodes = [start_node]

    for iteration in range(max_iter):
        # Sample with goal biasing
        x_rand, y_rand = sample_point(width, height, goal, goal_bias)
        if grid[invert_y(y_rand, height), x_rand] == 0:
            continue
            
        rand_node = Node(x_rand, y_rand)
        nearest_node = min(nodes, key=lambda node: distance(node, rand_node))
        
        # Adaptive step size (larger steps when far from goal)
        dist_to_goal = distance(nearest_node, goal_node)
        step_size = min(max(20, dist_to_goal/10), 50)
        
        dx = rand_node.x - nearest_node.x
        dy = rand_node.y - nearest_node.y
        dist = math.hypot(dx, dy)
        if dist == 0:
            continue
            
        scale = step_size / dist
        new_node = Node(
            int(nearest_node.x + dx * scale),
            int(nearest_node.y + dy * scale)
        )

        # Calculate yaw angle
        new_node.yaw = math.atan2(dy, dx)  # Calculate yaw angle from dx, dy

        # Check path validity
        if not is_collision_free((nearest_node.x, nearest_node.y), (new_node.x, new_node.y), grid):
            continue

        # Update costs and parent
        new_node.cost = nearest_node.cost + distance(nearest_node, new_node)
        new_node.parent = nearest_node

        # Rewire nearby nodes
        radius = min(base_radius * (1 + iteration/1000), 100)  # Gradually increase radius
        for node in nodes:
            if (distance(node, new_node) < radius and 
                node.cost > new_node.cost + distance(new_node, node) and 
                is_collision_free((new_node.x, new_node.y), (node.x, node.y), grid)):
                node.parent = new_node
                node.cost = new_node.cost + distance(new_node, node)

        nodes.append(new_node)

        # Dynamic goal checking
        goal_threshold = max(15, step_size/2)
        if (distance(new_node, goal_node) < goal_threshold and 
            is_collision_free((new_node.x, new_node.y), (goal_node.x, goal_node.y), grid)):
            goal_node.parent = new_node
            goal_node.cost = new_node.cost + distance(new_node, goal_node)
            nodes.append(goal_node)
            print(f"✓ Found path in {iteration} iterations")
            return nodes, goal_node

    print(f"⚠ Reached max iterations ({max_iter}) without finding path")
    return nodes, Node(goal[0], goal[1])  # Return best effort

def extract_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y, node.yaw))  # Append yaw here
        node = node.parent
    return path[::-1]

def draw_tree_and_path(img, nodes, goal_node, start_point, goal_point, step, followed_path):
    height = img.shape[0]
    img = img.copy()

    # Draw tree
    for node in nodes:
        y = invert_y(node.y, height)
        if node.parent:
            py = invert_y(node.parent.y, height)
            cv2.line(img, (node.x, y), (node.parent.x, py), (200, 100, 100), 1)
        cv2.circle(img, (node.x, y), 1, (255, 200, 0), -1)

    # Draw followed path
    for i in range(1, len(followed_path)):
        x1, y1, _ = followed_path[i - 1]  # Ignore yaw for path visualization
        x2, y2, _ = followed_path[i]
        y1 = invert_y(y1, height)
        y2 = invert_y(y2, height)
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # Draw start and goal
    cv2.circle(img, (start_point[0], invert_y(start_point[1], height)), 5, (0, 255, 0), -1)
    cv2.circle(img, (goal_point[0], invert_y(goal_point[1], height)), 5, (0, 165, 255), -1)
    
    # Add step counter
    cv2.putText(img, f"Step: {step}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return img

def save_path_to_file(path, filename="path.txt"):
    offset_x, offset_y, offset_yaw = -60, -60, 0
    with open(filename, "a") as file:
        for point in path:
            x, y, yaw = point
            x += offset_x
            y += offset_y
            if yaw == None:
                yaw = 0
            yaw += offset_yaw  # if ever needed
            file.write(f"{x}, {y}, {yaw}\n")
        file.write("\n")


def get_valid_point(prompt, grid):
    while True:
        try:
            coords = input(f"{prompt} (x y): ").strip()
            if not coords:
                print("Generating random valid point...")
                free_indices = np.argwhere(grid == 1)
                if len(free_indices) > 0:
                    x, y = random.choice(free_indices)
                    return (y, x)  # Note: numpy uses (row, col) = (y, x)
                else:
                    print("❌ No free space found in maze!")
                    continue
                    
            x, y = map(int, coords.split())
            if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
                inv_y = invert_y(y, grid.shape[0])
                if grid[inv_y, x] == 1:
                    return (x, y)
                print("❌ Point is inside obstacle")
            else:
                print(f"❌ Coordinates must be 0-{grid.shape[1]-1} x 0-{grid.shape[0]-1}")
        except ValueError:
            print("❌ Please enter two integers separated by space")

def get_random_free_target_nearby(current, grid, radius=100):
    for _ in range(100):  # More attempts
        angle = random.uniform(0, 2*math.pi)
        dist = random.uniform(radius/2, radius)
        dx = int(dist * math.cos(angle))
        dy = int(dist * math.sin(angle))
        new_x, new_y = current[0] + dx, current[1] + dy
        if (0 <= new_x < grid.shape[1] and 0 <= new_y < grid.shape[0] and
            grid[invert_y(new_y, grid.shape[0]), new_x] == 1):
            return (new_x, new_y)
    return current  # Fallback

def is_valid_point(point, grid):
    """Check if a point (x, y) is within bounds and in free space."""
    x, y = point
    if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
        inv_y = invert_y(y, grid.shape[0])
        return grid[inv_y, x] == 1
    return False

def run_planner(occupancy_grid, start, goal):
    # Get valid points (with random generation option)
    print("\n=== RRT* Path Planning ===")
    print("Click on the maze visualization to see coordinates")
    print("Press Enter at prompt to generate random point\n")
    
    # start_point = get_valid_point("Enter start point", occupancy_grid)
    # initial_goal = get_valid_point("Enter initial goal point", occupancy_grid)
    start_point = start
    initial_goal = goal
    is_valid_point(start, occupancy_grid)
    is_valid_point(goal, occupancy_grid)
    # Initialize tracking
    color_maze = cv2.cvtColor(binary_maze, cv2.COLOR_GRAY2BGR)
    frames = []
    goal_x, goal_y = initial_goal
    followed_path = [(start_point[0], start_point[1], 0.0)]  # Add yaw as 0 initially
    current_path = []
    current_path_index = 0
    goal_reached = False
    all_path = []

    # Main planning loop
    for step in range(100):  # Maximum simulation steps
        # Update goal position (dynamic movement using RRT*)
        if step % 5 == 0:  # Replan goal path more frequently
            goal_target = get_random_free_target_nearby((goal_x, goal_y), occupancy_grid, 40)
            g_nodes, g_goal = rrt_star((goal_x, goal_y), goal_target, occupancy_grid, max_iter=5000)
            if g_goal.parent:
                g_path = extract_path(g_goal)
                move_step = min(3, len(g_path)-1)
                goal_x, goal_y, _ = g_path[move_step]  # Ignore yaw for movement
    
        # Replan if needed (original RRT* planning)
        if current_path_index >= len(current_path):
            nodes, goal_node = rrt_star(followed_path[-1][:2], (goal_x, goal_y), occupancy_grid)  # Use the 2D part of the last point in followed_path
            if goal_node.parent:
                current_path = extract_path(goal_node)
                current_path_index = 1
                save_path_to_file(current_path)  # Save the new path to file
                print('This is siz eof all path',len(all_path))
                for p in current_path:
                    all_path.append(p)
            else:
                print("⚠ Can't find path to goal - waiting...")
                current_path_index = 0
                current_path = []
        
        # Move along path
        if current_path and current_path_index < len(current_path):
            followed_path.append((current_path[current_path_index][0], current_path[current_path_index][1], 0.0))  # Add yaw as 0.0 (can calculate later)
            current_path_index += 1
        
        # Check goal reached
        last_point = followed_path[-1]
        if math.hypot(last_point[0]-goal_x, last_point[1]-goal_y) < 100:
            print(f"🎯 Goal reached at step {step}!")
            goal_reached = True
        
        # Visualization
        frame = draw_tree_and_path(color_maze, nodes, goal_node, 
                                 followed_path[-1][:2], (goal_x, goal_y),  # Use only the 2D part for drawing
                                 step, followed_path)
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
        if goal_reached:
            break

    return all_path
# === Main Execution ===
if __name__ == "__main__":
    run_planner(occupancy_grid, (99,99), (1249, 700))

    # # Visualization setup
    # plt.rcParams['figure.figsize'] = [11, 6]
    
    # # Get valid points (with random generation option)
    # print("\n=== RRT* Path Planning ===")
    # print("Click on the maze visualization to see coordinates")
    # print("Press Enter at prompt to generate random point\n")
    
    # start_point = get_valid_point("Enter start point", occupancy_grid)
    # initial_goal = get_valid_point("Enter initial goal point", occupancy_grid)
    

    # all_path = []

    # # Initialize tracking
    # color_maze = cv2.cvtColor(binary_maze, cv2.COLOR_GRAY2BGR)
    # frames = []
    # goal_x, goal_y = initial_goal
    # followed_path = [(start_point[0], start_point[1], 0.0)]  # Add yaw as 0 initially
    # current_path = []
    # current_path_index = 0
    # goal_reached = False
    # step = 0
    # # Main planning loop
    # while True:  # Maximum simulation steps
    #     # Update goal position (dynamic movement using RRT*)
    #     if step % 5 == 0:  # Replan goal path more frequently
    #         goal_target = get_random_free_target_nearby((goal_x, goal_y), occupancy_grid, 300)
    #         g_nodes, g_goal = rrt_star((goal_x, goal_y), goal_target, occupancy_grid, max_iter=5000)
    #         if g_goal.parent:
    #             g_path = extract_path(g_goal)
    #             move_step = min(3, len(g_path)-1)
    #             goal_x, goal_y, _ = g_path[move_step]  # Ignore yaw for movement
    
    #     # Replan if needed (original RRT* planning)
    #     if current_path_index >= len(current_path):
    #         nodes, goal_node = rrt_star(followed_path[-1][:2], (goal_x, goal_y), occupancy_grid)  # Use the 2D part of the last point in followed_path
    #         if goal_node.parent:
    #             current_path = extract_path(goal_node)
    #             current_path_index = 1
    #             save_path_to_file(current_path)  # Save the new path to file
    #             print('This is siz eof all path',len(all_path))
    #             for p in current_path:
    #                 all_path.append(p)
    #         else:
    #             print("⚠ Can't find path to goal - waiting...")
    #             current_path_index = 0
    #             current_path = []
        
    #     # Move along path
    #     if current_path and current_path_index < len(current_path):
    #         followed_path.append((current_path[current_path_index][0], current_path[current_path_index][1], 0.0))  # Add yaw as 0.0 (can calculate later)
    #         current_path_index += 1
        
    #     # Check goal reached
    #     last_point = followed_path[-1]
    #     if math.hypot(last_point[0]-goal_x, last_point[1]-goal_y) < 100:
    #         print(f"🎯 Goal reached at step {step}!")
    #         goal_reached = True
        
    #     # Visualization
    #     frame = draw_tree_and_path(color_maze, nodes, goal_node, 
    #                              followed_path[-1][:2], (goal_x, goal_y),  # Use only the 2D part for drawing
    #                              step, followed_path)
    #     frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    #     step += 1
    #     if goal_reached:
    #         break



    # # p = run_planner(occupancy_grid)

    # print(p)
    # # Animation
    # print("\nGenerating animation...")
    # fig, ax = plt.subplots(figsize=(11, 6))
    # img_disp = ax.imshow(frames[0])
    # ax.axis("off")
    # title = ax.text(0.5, 0.95, "", transform=ax.transAxes, 
    #                ha="center", color='white',
    #                bbox=dict(facecolor='black', alpha=0.7))
    
    # def update(i):
    #     img_disp.set_data(frames[i])
    #     title.set_text(f"Step {i}")
    #     return [img_disp, title]
    
    # ani = animation.FuncAnimation(fig, update, frames=len(frames), 
    #                             interval=100, blit=True)
    # plt.tight_layout()
    # plt.show(block=True)
    # print("Animation displayed. Close the window to exit.")
    # print("Done!")
