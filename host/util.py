import numpy as np
    

def local_to_world(polar_points, pose):
    robot_x, robot_y, robot_theta = pose

    cos_h = np.cos(robot_theta)
    sin_h = np.sin(robot_theta)

    world_points = []

    for px, py in polar_points:

        wx = robot_x + (px * cos_h - py * sin_h)
        wy = robot_y + (px * sin_h + py * cos_h)

        world_points.append((wx, wy))

    return world_points

def get_pose_at_timestamp(history, timestamp):

    if len(history) < 2:
        return None

    # Too old
    if timestamp < history[0][0]:
        return None

    # Too new
    if timestamp > history[-1][0]:
        return None

    for i in range(len(history)-1):

        t0, pose0 = history[i]
        t1, pose1 = history[i+1]

        if t0 <= timestamp <= t1:

            # (target - time_start) / (time_end - time_start)
            alpha = (timestamp - t0) / (t1 - t0) 
            

            x = pose0[0] + alpha * (pose1[0] - pose0[0])
            y = pose0[1] + alpha * (pose1[1] - pose0[1])


            dtheta = np.arctan2(
                np.sin(pose1[2] - pose0[2]),
                np.cos(pose1[2] - pose0[2])
            )

            theta = pose0[2] + alpha * dtheta

            return np.array([x, y, theta])

    return None


def compose_pose(a, b):
    """
    Apply transform a, then pose b.

    a = [x, y, theta]
    b = [x, y, theta]
    """
    ax, ay, atheta = a
    bx, by, btheta = b

    c = np.cos(atheta)
    s = np.sin(atheta)

    x = ax + c * bx - s * by
    y = ay + s * bx + c * by
    theta = atheta + btheta

    theta = np.arctan2(np.sin(theta), np.cos(theta))

    return np.array([x, y, theta])


def inverse_pose(pose):
    x, y, theta = pose

    c = np.cos(theta)
    s = np.sin(theta)

    inv_x = -c * x - s * y
    inv_y =  s * x - c * y
    inv_theta = -theta

    return np.array([
        inv_x,
        inv_y,
        inv_theta
    ])