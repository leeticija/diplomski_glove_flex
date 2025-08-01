#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt

def plot_multiple_rosbag_topics(bag_path, topic_list):
    topic_data = {topic: {"time": [], "value": []} for topic in topic_list}

    print(f"Reading bag: {bag_path}")
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=topic_list):
            topic_data[topic]["time"].append(t.to_sec())
            topic_data[topic]["value"].append(msg.data)

    # Normalize time using the earliest timestamp
    all_times = [t for topic in topic_list for t in topic_data[topic]["time"]]
    if not all_times:
        print("No data found for given topics.")
        return

    start_time = min(all_times)
    for topic in topic_list:
        topic_data[topic]["time"] = [t - start_time for t in topic_data[topic]["time"]]

    # Plotting
    plt.figure(figsize=(10, 5))
    for topic in topic_list:
        if topic_data[topic]["time"]:
            plt.plot(topic_data[topic]["time"], topic_data[topic]["value"], label=topic)
        else:
            print(f"Warning: No data found on topic '{topic}'")

    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.title("ROS Bag Topic Plot")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

# plot_multiple_rosbag_topics("median.bag", ["/glove_data", "/glove_data_median"])
# plot_multiple_rosbag_topics("median10.bag", ["/glove_data", "/glove_data_median"])
# plot_multiple_rosbag_topics("omeaaaa.bag", ["/glove_data", "/glove_data_ome"])
plot_multiple_rosbag_topics("../bags/slow_close_gripper_data.bag", ["/gripper_data_ome"])