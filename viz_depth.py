from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import numpy as np
import cv2
import struct

bag_path = Path("bathroom/rosbag2_2025_10_20-16_47_22")
typestore = get_typestore(Stores.ROS2_HUMBLE)

depth_topic = "/zed/zed_node/depth/depth_registered/compressedDepth"

with AnyReader([bag_path], default_typestore=typestore) as reader:
    depth_count = 0

    for conn, timestamp, rawdata in reader.messages(connections=reader.connections):
        if conn.topic == depth_topic:
            msg = reader.deserialize(rawdata, conn.msgtype)
            depth_count += 1

            print(f"\nDepth Image {depth_count} Details:")
            print(f"  Format: {msg.format}")
            print(f"  Data size: {len(msg.data)} bytes")

            # Decode the ZED compressed depth format:
            # First 12 bytes are: rows (4), cols (4), type (4)
            header_size = 12
            if len(msg.data) > header_size:
                # Extract PNG bytes starting from offset 12
                png_bytes = msg.data[header_size:]
                np_arr = np.frombuffer(png_bytes, np.uint8)
                depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

                if depth_img is not None:
                    print(f"Depth Image {depth_count}: {depth_img.shape}, dtype={depth_img.dtype}")
                    print(f"Min={depth_img.min()}, Max={depth_img.max()}")

                    # Normalize for display
                    depth_vis = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
                    depth_vis = depth_vis.astype(np.uint8)
                    cv2.imshow("ZED Depth", depth_vis)
                    if cv2.waitKey(1) == 27:  # ESC to quit
                        break
                else:
                    print(f"⚠️ cv2.imdecode failed for image {depth_count}")
            else:
                print(f"⚠️ Not enough data in message {depth_count}")

            

    cv2.destroyAllWindows()
