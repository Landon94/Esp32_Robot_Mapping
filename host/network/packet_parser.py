import struct
from telemetry.messages import RPMData, LidarSampleData, LidarBatchData, FullScan

# --- STRUCT HEADER FORMAT ---
# '<' Little-Endian
# I = 4-byte enum (TelemetryType)
# I = 4-byte unsigned int (sequence)
# Q = 8-byte unsigned int (timestamp)
HEADER_FORMAT = "<IIQ"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

# -- STRUCT LIDAR FORMAT ---
# f = 4-byte float (angle)
# f = 4-byte float (distance)
# B = 1-byte unsigned char/int (quality)
# ? = 1-byte boolean (start flag)
# 2x = padding
LIDAR_FORMAT = "<ffB?2x"
LIDAR_FORMAT_SIZE = struct.calcsize(LIDAR_FORMAT)

# -- STRUCT LIDAR BATCH FORMAT ---
# uint16_t batch_count + 2 bytes alignment padding
BATCH_COUNT_FORMAT = "<H2x"
BATCH_COUNT_FORMAT_SIZE = struct.calcsize(BATCH_COUNT_FORMAT)


# -- STRUCT RPM FORMAT ---
# f = 4-byte float (left_rpm)
# f = 4-byte float (right_rpm)
RPM_FORMAT = "<ff"
RPM_FORMAT_SIZE = struct.calcsize(RPM_FORMAT)


def parse_data(data):

    header = data[:HEADER_SIZE]
    msg_type, sequence, timestamp_us = struct.unpack(HEADER_FORMAT, header)

    # Lidar = 0
    if msg_type == 0:
        return (0, _parse_lidar(sequence, timestamp_us, data))

    # RPM = 1
    elif msg_type == 1:
        return (1, _parse_rpm(sequence, timestamp_us, data))

    else:
        raise ValueError(f"Unknown telemetry type: {msg_type}")


def _parse_lidar(sequence, timestamp, data):

    batch_count, = struct.unpack_from(BATCH_COUNT_FORMAT, data, HEADER_SIZE)

    samples = []
    base_offset = HEADER_SIZE + BATCH_COUNT_FORMAT_SIZE

    for i in range(batch_count):

        # Calculate the byte offset of the current lidar sample
        lidar_sample_offset = base_offset + (i * LIDAR_FORMAT_SIZE)

        angle, distance, quality, start_flag = struct.unpack_from(LIDAR_FORMAT, data, lidar_sample_offset)

        sample = LidarSampleData(angle_deg=angle, distance_mm=distance, quality=quality, start_flag=start_flag)

        samples.append(sample)

    return LidarBatchData(sequence=sequence, timestamp_us=timestamp, samples=samples)


def _parse_rpm(sequence, timestamp, data):

    left_rpm, right_rpm = struct.unpack_from(RPM_FORMAT, data, HEADER_SIZE)

    return RPMData(sequence=sequence, timestamp_us=timestamp, left_rpm=left_rpm, right_rpm=right_rpm)

    
class LidarScanAssembler:

    def __init__(self, max_buffer_size):

        self.packet_buffer = {}

        self.max_sequence_num = 0
        self.max_buffer_size = max_buffer_size

    def add_batch(self, batch: LidarBatchData):

        self.packet_buffer[batch.sequence] = batch
        if batch.sequence > self.max_sequence_num:
            self.max_sequence_num = batch.sequence

        if len(self.packet_buffer) > self.max_buffer_size:
            self._remove_batches(min(self.packet_buffer.keys()))

        return len(self.packet_buffer)

    def _remove_batches(self, start_seq):
        del self.packet_buffer[start_seq]
        start_seq += 1

        while start_seq in self.packet_buffer:

            for sample in self.packet_buffer[start_seq].samples:

                if sample.start_flag:
                    return

            del self.packet_buffer[start_seq]
            start_seq += 1

    def build_full_scan(self):

        cur_seq = min(self.packet_buffer.keys())

        # Check the buffer has all needed packets (9 batches = 450 samples, full scan = 360 samples)
        has_all_packets = all(exp_seq in self.packet_buffer for exp_seq in range(cur_seq+1, cur_seq+10))

        if not has_all_packets:
            # If the cur batch is 30 batches away from newest, remove the batch group
            if abs(self.max_sequence_num - cur_seq) > 30:
                self._remove_batches(cur_seq)

            return None

        full_scan = FullScan(timestamp_us=0, samples=[])
        found_start = False
        found_end = False
        first_batch_index = cur_seq
        last_batch_index = None

        while not found_end:

            if cur_seq not in self.packet_buffer:
                self._remove_batches(first_batch_index)
                return None

            batch = self.packet_buffer[cur_seq]

            for sample in batch.samples:

                if not found_start and sample.start_flag:
                    full_scan.timestamp_us = batch.timestamp_us
                    found_start = True

                elif not found_start:
                    continue

                elif found_start and sample.start_flag:
                    found_end = True
                    last_batch_index = cur_seq
                    break

                full_scan.samples.append(sample)

            cur_seq += 1

        for i in range(first_batch_index, last_batch_index):
            del self.packet_buffer[i]

        return full_scan