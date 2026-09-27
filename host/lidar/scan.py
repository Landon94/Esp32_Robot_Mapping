import numpy as np
from telemetry.messages import FullScan, FullScanCartesian
from util import local_to_world


def scan_matching(pose, scan, grid):
    polar_points = LidarBatch_to_polar(scan)

    grid_probabilities = grid.probabilities()
    best_score = float("-inf")
    best_pose  = pose

    # Search for scan in area of +/- 0.30m
    x_offsets = np.arange(-0.30, 0.301, 0.05)
    y_offsets = np.arange(-0.30, 0.301, 0.05)
    theta_offsets = np.deg2rad(np.arange(-5, 6, 1))

    for dx in x_offsets:
        for dy in y_offsets:
            for dtheta in theta_offsets:

                theta = pose[2] + dtheta
                theta = np.arctan2(
                    np.sin(theta),
                    np.cos(theta)
                )

                new_pose = [pose[0] + dx, pose[1] + dy, theta]

                score = score_pose(new_pose, polar_points, grid, grid_probabilities)

                if score > best_score:
                    best_score = score
                    best_pose = new_pose

    return best_pose
        

def score_pose(pose, polar_points, grid, prob):
    score = 0.0
    num_points = 0

    endpoints = local_to_world(polar_points, pose)

    for ex, ey in endpoints:

        gx, gy = grid.world_to_grid_points(ex, ey)

        if not grid.valid_cell(gx, gy):
            continue

        score += prob[gy, gx]
        num_points += 1

    if num_points == 0:
        return float("-inf")

    return score / num_points



def LidarBatch_to_polar(data: FullScan):

    polar_points = []

    for sample in data.samples:

        radial_dist = sample.distance_mm / 1000.0
        theta = np.deg2rad(sample.angle_deg)

        x = radial_dist * np.cos(theta)
        y = radial_dist * np.sin(theta)

        polar_points.append([x,y])

    return polar_points

