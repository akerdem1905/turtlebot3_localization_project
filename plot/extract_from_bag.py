import os
import csv
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.serde import deserialize_cdr

bag_path = os.path.expanduser('../rosbag2_2025_07_01-17_41_20')

topics_to_extract = {
    "/odom": "odom.csv",
    "/prediction": "prediction.csv",
    "/kf_prediction": "kf_prediction.csv",
    "/ekf_state": "ekf.csv",
}

def extract_pose(msg):
    try:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        return t, x, y
    except Exception:
        return None

with AnyReader([Path(bag_path)]) as reader:
    writers = {}
    for topic, fname in topics_to_extract.items():
        f = open(fname, 'w', newline='')
        writer = csv.writer(f)
        writer.writerow(['t', 'x', 'y'])
        writers[topic] = writer

    for connection, timestamp, rawdata in reader.messages():
        topic = connection.topic
        if topic in topics_to_extract:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            result = extract_pose(msg)
            if result:
                writers[topic].writerow(result)

print("✅ Extraktion abgeschlossen:")
for f in topics_to_extract.values():
    print(" -", f)
