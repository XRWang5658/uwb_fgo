import rosbag
import pandas as pd

# Paths to the bag file and output CSV files
bag_file = "/home/xiangru/code/uwb_ws/src/uwb_fgo/data/fosion_uwb.bag"
baseline_csv = "baseline_path.csv"

# Topics to extract
baseline_topic = "/uwb_pose"  # Baseline topic

# Function to extract path data from a topic
def extract_pose_stamped(bag, topic, output_csv):
    """
    Extracts data from a topic publishing geometry_msgs/PoseStamped messages.
    """
    data = {"timestamp": [], "x": [], "y": [], "z": []}
    for _, msg, t in bag.read_messages(topics=[topic]):
        data["timestamp"].append(msg.header.stamp.to_sec())
        data["x"].append(msg.pose.position.x)
        data["y"].append(msg.pose.position.y)
        data["z"].append(msg.pose.position.z)
    df = pd.DataFrame(data)
    df.to_csv(output_csv, index=False)
    print(f"Extracted {len(df)} poses from {topic} to {output_csv}")

def extract_path(bag, topic, output_csv):
    """
    Extracts data from a topic publishing nav_msgs/Path messages.
    """
    data = {"timestamp": [], "x": [], "y": [], "z": []}
    for _, msg, t in bag.read_messages(topics=[topic]):
        for pose in msg.poses:
            data["timestamp"].append(pose.header.stamp.to_sec())
            data["x"].append(pose.pose.position.x)
            data["y"].append(pose.pose.position.y)
            data["z"].append(pose.pose.position.z)
    df = pd.DataFrame(data)
    df.to_csv(output_csv, index=False)
    print(f"Extracted {len(df)} poses from {topic} to {output_csv}")

# Open the bag file and extract data
with rosbag.Bag(bag_file, "r") as bag:
    
    # Extract baseline path (if it's a geometry_msgs/PoseStamped topic)
    extract_pose_stamped(bag, baseline_topic, baseline_csv)