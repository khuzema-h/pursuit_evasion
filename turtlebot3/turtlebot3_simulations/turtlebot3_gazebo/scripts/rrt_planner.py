import random
import math
import numpy as np

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRTPlanner:
    def __init__(self, occupancy_grid, width, height, resolution, origin, max_iter=500, step_size=0.2):
        self.map = np.array(occupancy_grid).reshape((height, width))
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin  # (x, y)
        self.max_iter = max_iter
        self.step_size = step_size

    def plan(self, start, goal):
        start_node = Node(*start)
        goal_node = Node(*goal)

        nodes = [start_node]

        for i in range(self.max_iter):
            rand_point = self.sample_free()

            nearest_node = self.get_nearest_node(nodes, rand_point)

            new_node = self.steer(nearest_node, rand_point)

            if self.is_collision_free(nearest_node, new_node):
                nodes.append(new_node)

                if self.distance((new_node.x, new_node.y), (goal_node.x, goal_node.y)) < self.step_size:
                    goal_node.parent = new_node
                    nodes.append(goal_node)
                    return self.retrace_path(goal_node)

        print("Failed to find a path")
        return None

    def sample_free(self):
        while True:
            x = random.uniform(0, self.width * self.resolution) + self.origin[0]
            y = random.uniform(0, self.height * self.resolution) + self.origin[1]
            if not self.is_occupied(x, y):
                return (x, y)

    def get_nearest_node(self, nodes, point):
        return min(nodes, key=lambda node: self.distance((node.x, node.y), point))

    def steer(self, from_node, to_point):
        theta = math.atan2(to_point[1] - from_node.y, to_point[0] - from_node.x)
        new_x = from_node.x + self.step_size * math.cos(theta)
        new_y = from_node.y + self.step_size * math.sin(theta)

        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        return new_node

    def is_collision_free(self, from_node, to_node):
        steps = int(self.distance((from_node.x, from_node.y), (to_node.x, to_node.y)) / (self.step_size / 2))
        for i in range(steps):
            x = from_node.x + (to_node.x - from_node.x) * i / steps
            y = from_node.y + (to_node.y - from_node.y) * i / steps
            if self.is_occupied(x, y):
                return False
        return True

    def is_occupied(self, x, y):
        map_x = int((x - self.origin[0]) / self.resolution)
        map_y = int((y - self.origin[1]) / self.resolution)

        if map_x < 0 or map_x >= self.width or map_y < 0 or map_y >= self.height:
            return True

        return self.map[map_y][map_x] > 50  # occupied if >50 (standard occupancy grid threshold)

    def retrace_path(self, goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

    def distance(self, point1, point2):
        return math.hypot(point1[0] - point2[0], point1[1] - point2[1])
