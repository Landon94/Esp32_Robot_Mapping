import numpy as np

class OccupancyGrid:

    def __init__(self, width_m, height_m, resolution):
        self.resolution = resolution

        self.width = int(width_m / resolution)
        self.height = int(height_m / resolution)

        # Bottom left point of grid  (0,0)
        self.origin_x = -width_m / 2.0
        self.origin_y = -height_m / 2.0

        self.grid = np.zeros((self.height, self.width), dtype=np.float32)

        self.free_update = -0.4
        self.occupied_update = 0.85

        self.min_log_odds = -5.0
        self.max_log_odds = 5.0

    def world_to_grid_points(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)

        return gx, gy

    def general_bresenham(self, x1, y1, x2, y2):
        
        points = []

        # Distance between points
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        # Direction of the slop
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1

        # Start points
        x = x1
        y = y1

        # If line more horizontal take horizontal step
        if dx > dy:

            err = dx / 2

            while x != x2:
                points.append((x, y))

                err -= dy

                # If we have drifted far enougth take a step in y
                if err < 0:
                    y += sy
                    err += dx

                # Take a step in x direction
                x += sx

        # dy > dx, slope is to step need to step with y to keep up
        else:

            err = dy / 2

            while y != y2:
                points.append((x, y))

                err -= dx

                if err < 0:
                    x += sx
                    err += dy

                y += sy

        points.append((x2, y2))

        return points

    def valid_cell(self, gx, gy):
        return (0 <= gx < self.width and 0 <= gy < self.height)


    def update(self, robot_pose, world_points):
        robot_x, robot_y, robot_theta = robot_pose


        robot_gx, robot_gy = self.world_to_grid_points(robot_x, robot_y)


        for x, y in world_points:

            gx, gy = self.world_to_grid_points(x, y)

            if not self.valid_cell(gx, gy):
                continue

            points = self.general_bresenham(robot_gx, robot_gy, gx, gy)

            for gx ,gy in points[:-1]:
                if self.valid_cell(gx, gy):
                    self.grid[gy, gx] += self.free_update

            gx, gy = points[-1]
            if self.valid_cell(gx, gy):
                self.grid[gy, gx] += self.occupied_update

        np.clip(
            self.grid,
            self.min_log_odds,
            self.max_log_odds,
            out=self.grid
        )

    def probabilities(self):
        return 1.0 / (1.0 + np.exp(-self.grid))