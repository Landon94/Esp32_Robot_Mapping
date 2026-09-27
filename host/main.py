import rerun as rr
import numpy as np

from collections import deque

from network.packet_parser import LidarScanAssembler, parse_data
from network.udp_receiver import ReceiverUDP
from lidar.scan import LidarBatch_to_polar, scan_matching
from SLAM.occupancy_grid import OccupancyGrid
from SLAM.odemetry import Odemetry
from util import get_pose_at_timestamp, local_to_world, compose_pose, inverse_pose


GRID_WIDTH_M    = 30
GRID_HEIGHT_M   = 30
GRID_RESOLUTION = 0.05


def process_scan(scan, pose, grid):
    polar_points = LidarBatch_to_polar(scan)

    world_points = local_to_world(polar_points, pose)

    grid.update(pose, world_points)
    
    rr.log("world/lidar", rr.Points2D(world_points, radii=0.01))

    rr.log("world/robot",rr.Points2D([[pose[0], pose[1]]],radii=0.08))

    # Log occupancy map
    probabilities = grid.probabilities()

    image = (probabilities * 255).astype(np.uint8)

    rr.log("map",rr.Image(image))

def run():

    rr.init("robot_slam", spawn=True)

    msgReceiver   = ReceiverUDP()
    scanAssembler = LidarScanAssembler(max_buffer_size=100)
    wheelOdemetry = Odemetry()
    grid          = OccupancyGrid(GRID_WIDTH_M, GRID_HEIGHT_M, GRID_RESOLUTION)


    current_pose = np.array([0.0, 0.0, 0.0])
    map_odom_correction = np.array([0.0, 0.0, 0.0])
    pose_history = deque(maxlen=1000)
    pending_scans = deque()

    has_map = False

    try:
        while True:
            # recvfrom blocks until a packet arrives
            data, addr = msgReceiver.receive_udp()

            data_size = len(data)
            print(f"Received {data_size} bytes from {addr}")
            
            # # Decode the byte payload into a readable string
            msg_type, data_class = parse_data(data)

            # Lidar msg
            if msg_type == 0:
                num_of_batches = scanAssembler.add_batch(data_class)
                if num_of_batches >= 10:
                    scan = scanAssembler.build_full_scan()

                    if scan is None:
                        continue

                    timestamp = scan.timestamp_us

                    # pose = get_pose_at_timestamp(pose_history, timestamp)

                    # if pose is not None:

                    #     if has_map:
                    #         pose = scan_matching(pose, scan, grid)

                    #     has_map = True
                    #     process_scan(scan, pose, grid)
                    odom_pose = get_pose_at_timestamp(pose_history,timestamp)

                    if odom_pose is not None:

                        predicted_pose = compose_pose(map_odom_correction, odom_pose)

                        if has_map:
                            corrected_pose = scan_matching(predicted_pose, scan, grid)

                            map_odom_correction = compose_pose(corrected_pose, inverse_pose(odom_pose))

                        else:
                            corrected_pose = predicted_pose

                        process_scan(scan, corrected_pose, grid)

                        has_map = True

                    else:

                        # No odometry yet
                        if len(pose_history) == 0:
                            pending_scans.append(scan)

                        # Check if scan to old then drop
                        elif timestamp < pose_history[0][0]:
                            pass

                        else:
                            pending_scans.append(scan)


            # Rpm msg
            elif msg_type == 1:
                
                seq = data_class.sequence
                timestamp = data_class.timestamp_us
                left_rpm = data_class.left_rpm
                right_rpm = data_class.right_rpm

                current_pose = wheelOdemetry.update(timestamp, right_rpm, left_rpm)

                pose_history.append((timestamp, current_pose))

                # Try pending scans again
                while pending_scans:

                    scan = pending_scans[0]

                    # pose = get_pose_at_timestamp(
                    #     pose_history,
                    #     scan.timestamp_us
                    # )

                    # if pose is not None:
                    #     pending_scans.popleft()

                    #     if has_map:
                    #         pose = scan_matching(pose, scan, grid)

                    #     has_map = True
                    #     process_scan(scan, pose, grid)

                    odom_pose = get_pose_at_timestamp(pose_history,timestamp)

                    if odom_pose is not None:

                        predicted_pose = compose_pose(map_odom_correction, odom_pose)

                        if has_map:
                            corrected_pose = scan_matching(predicted_pose, scan, grid)

                            map_odom_correction = compose_pose(corrected_pose, inverse_pose(odom_pose))

                        else:
                            corrected_pose = predicted_pose

                        process_scan(scan, corrected_pose, grid)

                        has_map = True

                    # Too old now, discard it
                    elif scan.timestamp_us < pose_history[0][0]:
                        pending_scans.popleft()

                    else:
                        # Oldest pending scan is still newer than
                        # our latest pose, so later scans will be too.
                        break

        
    except KeyboardInterrupt:
        print("\nServer shutting down gracefully.")
    
    finally:
        msgReceiver.shutdown_udp()
        rr.rerun_shutdown()


def main():
    run()


if __name__ == "__main__":
    main()