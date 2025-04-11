import json
import numpy as np
from random import uniform
import matplotlib.pyplot as plt

class PathPlanner:
    def __init__(self, obstacles):
        self.obstacles = obstacles

    def is_in_collision(self, point):
        """Kiểm tra xem điểm có nằm trong vật cản không"""
        x, y, z = point
        for obs in self.obstacles:
            if (obs["x_min"] <= x <= obs["x_max"] and 
                obs["y_min"] <= y <= obs["y_max"] and 
                obs["z_min"] <= z <= obs["z_max"]):
                return True
        return False
    
    def is_path_in_collision(self, p1, p2, step=0.5):
        """Kiểm tra toàn bộ đường đi từ p1 đến p2 có vướng vật cản không"""
        vec = np.array(p2) - np.array(p1)
        norm = np.linalg.norm(vec)
        if norm < step:
            return self.is_in_collision(p2)
        num_steps = int(norm / step)
        for i in range(1, num_steps + 1):
            inter_point = tuple(np.array(p1) + vec * (i / num_steps))
            if self.is_in_collision(inter_point):
                return True
        return False

    def random_point(self):
        """Tạo một điểm ngẫu nhiên không nằm trong vật cản"""
        while True:
            point = (uniform(-10, 10), uniform(-10, 10), uniform(0, 20))
            if not self.is_in_collision(point):
                return point

    def distance(self, p1, p2):
        """Tính khoảng cách Euclidean giữa hai điểm"""
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def steer(self, from_point, to_point, step=1.0):
        """Dẫn đường từ một điểm về phía điểm khác, với bước tối đa là `step`"""
        vec = np.array(to_point) - np.array(from_point)
        norm = np.linalg.norm(vec)
        if norm > 1e-6:  # Tránh lỗi chia cho 0
            return tuple(np.array(from_point) + vec / norm * step) if norm > step else to_point
        return from_point  # Nếu hai điểm trùng nhau, không di chuyển

    def retrace_path(self, tree, target):
        """Truy vết đường đi từ một cây"""
        path = []
        while target is not None and target in tree:
            path.append(target)
            target = tree[target]  # Lấy điểm cha
        return path[::-1]  # Đảo ngược để có đường đi đúng thứ tự

    def build_path(self, tree_start, tree_goal, point_start, point_goal):
        """Xây dựng đường đi từ hai cây"""
        path_start = self.retrace_path(tree_start, point_start)
        path_goal = self.retrace_path(tree_goal, point_goal)
        return path_start + list(reversed(path_goal))
    
    def simplify_path(self, path):
        """Rút gọn đường đi"""
        if not path:
            return []

        simplified_path = [path[0]]  # Bắt đầu với điểm đầu tiên
        current_idx = 0

        for next_idx in range(1, len(path)):
            if self.is_path_in_collision(path[current_idx], path[next_idx]):
                # Nếu có va chạm, giữ lại điểm ngay trước đó
                simplified_path.append(path[next_idx - 1])
                current_idx = next_idx - 1  # Cập nhật điểm hiện tại

        # Thêm điểm cuối
        simplified_path.append(path[-1])

        return simplified_path
    
    def densify_path(self, path, factor):
        """Tang mat do diem"""
        dense_path = []
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i + 1])
            
            # Tạo các điểm nội suy tuyến tính
            interp_points = np.linspace(p1, p2, factor + 2)[1:-1]  # Bỏ điểm đầu & cuối
            
            # Thêm điểm vào đường đi mới
            dense_path.append(tuple(p1))
            dense_path.extend(map(tuple, interp_points))

        # Thêm điểm cuối cùng
        dense_path.append(tuple(path[-1]))

        return dense_path

    def bi_RRT(self, start, goal, max_iter=50000):
        """Thuật toán Bi-RRT"""
        tree_start = {start: None}
        tree_goal = {goal: None}

        for _ in range(max_iter):
            # Mỗi cây lấy một điểm ngẫu nhiên khác nhau
            rand_point_start = self.random_point()
            rand_point_goal = self.random_point()

            # Tìm điểm gần nhất trong mỗi cây
            nearest_start = min(tree_start, key=lambda p: self.distance(p, rand_point_start))
            nearest_goal = min(tree_goal, key=lambda p: self.distance(p, rand_point_goal))

            # Mở rộng mỗi cây về điểm ngẫu nhiên của nó
            new_start = self.steer(nearest_start, rand_point_start)
            new_goal = self.steer(nearest_goal, rand_point_goal)

            # Kiểm tra toàn bộ đường đi trước khi thêm điểm mới
            if not self.is_path_in_collision(nearest_start, new_start):
                tree_start[new_start] = nearest_start
            if not self.is_path_in_collision(nearest_goal, new_goal):
                tree_goal[new_goal] = nearest_goal

            if self.distance(new_start, new_goal) < 2.0:
                if not self.is_path_in_collision(new_start, new_goal):
                    return self.build_path(tree_start, tree_goal, new_start, new_goal)

        return []
