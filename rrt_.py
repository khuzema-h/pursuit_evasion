import cv2
import numpy as np
import math
import random
from matplotlib import pyplot as plt
from matplotlib import animation

# === Maze loading ===
image_path = 'maze.png'
maze = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
if maze is None:
    raise FileNotFoundError(f"Could not load image at path: {image_path}")
_, binary_maze = cv2.threshold(maze, 127, 255, cv2.THRESH_BINARY)
occupancy_grid = (binary_maze == 255).astype(np.uint8)
height, width = occupancy_grid.shape

def invert_y(y, height): return height - 1 - y

class Node:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.parent = None
        self.cost = 0.0

def distance(n1, n2): return math.hypot(n1.x - n2.x, n1.y - n2.y)

def sample_point(width, height, goal, goal_bias=0.1):
    return goal if random.random() < goal_bias else (random.randint(0, width - 1), random.randint(0, height - 1))

def is_collision_free(p1, p2, grid):
    x1, y1 = p1
    x2, y2 = p2
    points = zip(np.linspace(x1, x2, 100).astype(int), np.linspace(y1, y2, 100).astype(int))
    return all(
        0 <= x < grid.shape[1] and 0 <= y < grid.shape[0] and grid[invert_y(y, grid.shape[0]), x] == 1
        for x, y in points
    )

def rrt_star(start, goal, grid, max_iter=3000, radius=15, goal_bias=0.1):
    height, width = grid.shape
    start_node = Node(*start)
    goal_node = Node(*goal)
    nodes = [start_node]

    for _ in range(max_iter):
        x_rand, y_rand = sample_point(width, height, goal, goal_bias)
        if grid[invert_y(y_rand, height), x_rand] == 0:
            continue
        rand_node = Node(x_rand, y_rand)

        nearest_node = min(nodes, key=lambda node: distance(node, rand_node))
        dx, dy = rand_node.x - nearest_node.x, rand_node.y - nearest_node.y
        dist = math.hypot(dx, dy)
        if dist == 0:
            continue

        scale = min(10.0 / dist, 1.0)
        new_node = Node(
            int(nearest_node.x + dx * scale),
            int(nearest_node.y + dy * scale)
        )

        if not is_collision_free((nearest_node.x, nearest_node.y), (new_node.x, new_node.y), grid):
            continue

        new_node.cost = nearest_node.cost + distance(nearest_node, new_node)
        new_node.parent = nearest_node

        for node in nodes:
            if distance(node, new_node) < radius and \
               node.cost > new_node.cost + distance(new_node, node) and \
               is_collision_free((new_node.x, new_node.y), (node.x, node.y), grid):
                node.parent = new_node
                node.cost = new_node.cost + distance(new_node, node)

        nodes.append(new_node)

        if distance(new_node, goal_node) < 10 and is_collision_free((new_node.x, new_node.y), (goal_node.x, goal_node.y), grid):
            goal_node.parent = new_node
            goal_node.cost = new_node.cost + distance(new_node, goal_node)
            nodes.append(goal_node)
            break

    return nodes, goal_node

def extract_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def draw_tree_and_path(img, nodes, goal_node, start_point, goal_point, step, followed_path):
    height = img.shape[0]
    img = img.copy()

    for node in nodes:
        y = invert_y(node.y, height)
        py = invert_y(node.parent.y, height) if node.parent else None
        cv2.circle(img, (node.x, y), 1, (255, 255, 0), -1)
        if node.parent:
            cv2.line(img, (node.x, y), (node.parent.x, py), (255, 0, 0), 1)

    for i in range(1, len(followed_path)):
        x1, y1 = followed_path[i - 1]
        x2, y2 = followed_path[i]
        y1 = invert_y(y1, height)
        y2 = invert_y(y2, height)
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

    cv2.circle(img, (start_point[0], invert_y(start_point[1], height)), 2, (0, 255, 0), -1)
    cv2.circle(img, (goal_point[0], invert_y(goal_point[1], height)), 2, (0, 255, 255), -1)
    return img

def get_valid_point(prompt, grid):
    while True:
        try:
            x, y = map(int, input(f"{prompt} (x y): ").split())
            if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
                inv_y = invert_y(y, grid.shape[0])
                if grid[inv_y, x] == 1:
                    return (x, y)
                else:
                    print("❌ That point is inside an obstacle.")
            else:
                print("❌ Coordinates out of bounds.")
        except ValueError:
            print("❌ Invalid input.")

def get_random_free_target_nearby(current, grid, radius=20):
    for _ in range(30):
        dx = random.randint(-radius, radius)
        dy = random.randint(-radius, radius)
        new_x, new_y = current[0] + dx, current[1] + dy
        if 0 <= new_x < grid.shape[1] and 0 <= new_y < grid.shape[0]:
            if grid[invert_y(new_y, grid.shape[0]), new_x] == 1:
                return (new_x, new_y)
    return current  # If nothing found, return current

# === MAIN ===
start_point = get_valid_point("Enter start point", occupancy_grid)
initial_goal = get_valid_point("Enter initial goal point", occupancy_grid)
goal_x, goal_y = initial_goal
color_maze = cv2.cvtColor(binary_maze, cv2.COLOR_GRAY2BGR)
frames = []
titles = []
step = 0
followed_path = [start_point]

# === Initial static RRT* plan ===
nodes, goal_node = rrt_star(start_point, initial_goal, occupancy_grid, max_iter=3000)
if goal_node.parent is None:
    print("❌ Initial RRT* plan failed.")
    exit()

path = extract_path(goal_node)
current_path_index = 1
goal_reached = False

# === Dynamic pursuit loop with valid RRT* goal movement ===
while not goal_reached:
    # Move goal through free space using short RRT* plan
    goal_target = get_random_free_target_nearby((goal_x, goal_y), occupancy_grid, radius=20)
    g_nodes, g_goal = rrt_star((goal_x, goal_y), goal_target, occupancy_grid, max_iter=200)
    if g_goal.parent:
        g_path = extract_path(g_goal)
        steps = min(3, len(g_path) - 1)
        if steps > 0:
            goal_x, goal_y = g_path[steps]

    goal = (goal_x, goal_y)

    if current_path_index < len(path):
        start_point = path[current_path_index]
        current_path_index += 1
    else:
        nodes, goal_node = rrt_star(start_point, goal, occupancy_grid, max_iter=100)
        if goal_node.parent:
            path = extract_path(goal_node)
            current_path_index = 1

    followed_path.append(start_point)
    frame = draw_tree_and_path(color_maze, nodes, goal_node, start_point, goal, step, followed_path)
    frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    titles.append(f"Step {step}")

    if math.hypot(start_point[0] - goal[0], start_point[1] - goal[1]) < 5:
        print(f"🎯 Reached moving goal at step {step}")
        goal_reached = True

    step += 1

# === Animate ===
fig, ax = plt.subplots()
img_disp = ax.imshow(frames[0])
step_text = ax.text(5, 10, titles[0], color='white', fontsize=12,
                   bbox=dict(facecolor='black', alpha=0.5, edgecolor='none'))
ax.axis("off")

def update(i):
    img_disp.set_data(frames[i])
    step_text.set_text(titles[i])
    return [img_disp, step_text]

ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=300, blit=True, repeat=False)
plt.show()
