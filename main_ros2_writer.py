import csv
import math
import os
import shutil
import sys

import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String


def latlon_to_enu(lat, lon, ref_lat, ref_lon):
    """Converts WGS-84 Lat/Lon coordinates to local East-North-Up (ENU) Cartesian meters."""
    R = 6371000.0  # Earth's mean radius in meters
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    x = dlon * math.cos(math.radians(ref_lat)) * R
    y = dlat * R
    return x, y


def write_ros2_bag(raw_rows, output_bag_directory="spray_mission_bag"):
    if not raw_rows:
        print("[ERROR] No telemetry data rows provided.")
        return

    # Delete existing bag directory to avoid overwrite conflict
    if os.path.exists(output_bag_directory):
        shutil.rmtree(output_bag_directory)

    # Determine reference origin (first valid non-zero GPS coordinate)
    ref_lat, ref_lon = None, None
    for row in raw_rows:
        try:
            lat = float(row.get("lat", 0.0))
            lon = float(row.get("lon", 0.0))
            if lat != 0.0 and lon != 0.0:
                ref_lat, ref_lon = lat, lon
                break
        except (ValueError, TypeError):
            continue

    if ref_lat is None:
        ref_lat, ref_lon = 0.0, 0.0

    # ROS 2 Bag Writer Initialization
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(
        uri=output_bag_directory, storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer.open(storage_options, converter_options)

    # Register ROS 2 Topics (ID, Topic Name, Message Type, Format)
    topics = [
        (1, "/drone/telemetry", "geometry_msgs/msg/PoseStamped", "cdr"),
        (2, "/spray_system/flow_rate", "std_msgs/msg/Float32", "cdr"),
        (3, "/spray_system/motor_rpm", "std_msgs/msg/Float32", "cdr"),
        (4, "/spray_system/cc_status", "std_msgs/msg/String", "cdr"),
        (5, "/spray_system/time_ist", "std_msgs/msg/String", "cdr"),
    ]

    for topic_id, topic_name, topic_type, topic_format in topics:
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                topic_id, topic_name, topic_type, topic_format
            )
        )

    base_nsec = 1700000000000000000  # Base Unix timestamp

    for idx, row in enumerate(raw_rows):
        lat = float(row.get("lat", ref_lat))
        lon = float(row.get("lon", ref_lon))
        heading = float(row.get("heading_deg", 0.0))
        flow_rate = float(row.get("flow_rate_ml_min", 0.0))
        rpm = float(row.get("spray_motor_rpm", 0.0))
        status = str(row.get("status", "NOMINAL"))
        time_ist = str(row.get("time_ist", ""))

        x, y = latlon_to_enu(lat, lon, ref_lat, ref_lon)
        frame_time_nsec = base_nsec + int(idx * 1e9)
        sec = int(frame_time_nsec // 1e9)
        nanosec = int(frame_time_nsec % 1e9)

        # 1. PoseStamped Message
        pose = PoseStamped()
        pose.header.stamp.sec = sec
        pose.header.stamp.nanosec = nanosec
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y

        yaw_rad = math.radians(heading)
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        writer.write(
            "/drone/telemetry", serialize_message(pose), frame_time_nsec
        )

        # 2. Flow Rate Message
        writer.write(
            "/spray_system/flow_rate",
            serialize_message(Float32(data=flow_rate)),
            frame_time_nsec,
        )

        # 3. Motor RPM Message
        writer.write(
            "/spray_system/motor_rpm",
            serialize_message(Float32(data=rpm)),
            frame_time_nsec,
        )

        # 4. CC Status String Message
        writer.write(
            "/spray_system/cc_status",
            serialize_message(String(data=status)),
            frame_time_nsec,
        )

        # 5. IST Timestamp Message
        writer.write(
            "/spray_system/time_ist",
            serialize_message(String(data=time_ist)),
            frame_time_nsec,
        )

    print(
        f"[SUCCESS] ROS 2 Bag created at '{output_bag_directory}' ({len(raw_rows)} records)."
    )


def main():
    csv_file = (
        sys.argv[1] if len(sys.argv) > 1 else "spray_mission_assessment.csv"
    )
    output_bag = sys.argv[2] if len(sys.argv) > 2 else "spray_mission_bag"

    if not os.path.exists(csv_file):
        print(f"[ERROR] Telemetry CSV file '{csv_file}' not found.")
        sys.exit(1)

    print(f"Reading CSV telemetry from: {csv_file}")
    with open(csv_file, "r", encoding="utf-8") as f:
        raw_rows = list(csv.DictReader(f))

    rclpy.init()
    try:
        write_ros2_bag(raw_rows, output_bag_directory=output_bag)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
