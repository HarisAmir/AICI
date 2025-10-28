from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import numpy as np
import cv2


bag_path = Path("office/rosbag2_2025_10_20-16_09_39")
typestore = get_typestore(Stores.ROS2_HUMBLE)

with AnyReader([bag_path], default_typestore=typestore) as reader:
    print("\nAvailable topics:")
   
    rgb_topic = "/zed/zed_node/rgb/image_rect_color/compressed"
    depth_topic = "/zed/zed_node/depth/depth_registered/compressedDepth"
    rgb_count = 0
    depth_count = 0

    for conn, timestamp, rawdata in reader.messages(connections=reader.connections):
     
        if conn.topic == rgb_topic:
            msg = reader.deserialize(rawdata, conn.msgtype)
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_count += 1
            print(f"RGB Image {rgb_count}: {img.shape}")
            if img is not None:
                cv2.imshow("ZED RGB Image", img)
                if cv2.waitKey(1) == 27:  # ESC to quit
                    break
            else:
                print("⚠️ Failed to decode RGB image data.")
            
    print(f"\nTotal RGB images processed: {rgb_count}")
   
    cv2.destroyAllWindows()
