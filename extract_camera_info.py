#!/usr/bin/env python3
"""Extract CameraInfo from rosbag2 by playing and subscribing."""
import sys, time, threading, sqlite3, struct

def parse_camera_info(data):
    """Parse sensor_msgs/CameraInfo from CDR bytes."""
    pos = 0
    # Header
    pos = 12
    # frame_id
    flen = struct.unpack_from('<I', data, pos)[0]; pos += 4
    frame_id = data[pos:pos+flen-1].decode(errors='replace'); pos += flen
    pos = (pos + 3) // 4 * 4  # align to 4

    # height, width
    height = struct.unpack_from('<I', data, pos)[0]; pos += 4
    width = struct.unpack_from('<I', data, pos)[0]; pos += 4

    # distortion_model
    dlen = struct.unpack_from('<I', data, pos)[0]; pos += 4
    dist_model = data[pos:pos+dlen-1].decode(errors='replace'); pos += dlen
    pos = (pos + 7) // 8 * 8  # align to 8

    # K[9]
    K = list(struct.unpack_from('<9d', data, pos)); pos += 72
    # D[5]
    D = list(struct.unpack_from('<5d', data, pos)); pos += 40
    # R[9]
    R = list(struct.unpack_from('<9d', data, pos)); pos += 72
    # P[12]
    P = list(struct.unpack_from('<12d', data, pos)); pos += 96

    fx_p = P[0]; fy_p = P[5]; cx_p = P[3]; cy_p = P[6]
    return {
        'width': width, 'height': height,
        'distortion_model': dist_model,
        'K': K,
        'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5],
        'D': D,
        'k1': D[0], 'k2': D[1], 'p1': D[2], 'p2': D[3], 'k3': D[4],
        'R': R,
        'P': P,
        'P_fx': fx_p, 'P_fy': fy_p, 'P_cx': cx_p, 'P_cy': cy_p,
    }

# Read first CameraInfo from bag
bag_path = '/home/lxy/asr_sdm_ws/datasheet/airground_rig_s3_ros2/airground_rig_s3_ros2.db3'
conn = sqlite3.connect(bag_path)
cur = conn.cursor()
cur.execute("SELECT id FROM topics WHERE name LIKE '%camera_info%'")
rows = cur.fetchall()
print(f"Camera info topics: {rows}")
if not rows:
    print("No camera_info topic found!")
    sys.exit(1)

topic_id = rows[0][0]
cur.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp LIMIT 1", (topic_id,))
msg_data = cur.fetchone()
if not msg_data:
    print("No messages found!")
    sys.exit(1)

info = parse_camera_info(bytes(msg_data[0]))
print(f"\nCameraInfo:")
print(f"  width={info['width']}, height={info['height']}")
print(f"  distortion_model='{info['distortion_model']}'")
print(f"  K = {[f'{x:.8f}' for x in info['K']]}")
print(f"  D = {[f'{x:.8f}' for x in info['D']]}")
print(f"  P = {[f'{x:.8f}' for x in info['P']]}")

# Check if K values are plausible (in pixel units, fx ~ 380-390, cx ~ 340-360)
if abs(info['K'][0]) > 100:  # likely in pixels
    print(f"\n[INTERPRETATION] K appears to be in PIXEL units:")
    print(f"  cam_fx: {info['K'][0]}")
    print(f"  cam_fy: {info['K'][4]}")
    print(f"  cam_cx: {info['K'][2]}")
    print(f"  cam_cy: {info['K'][5]}")
    print(f"  cam_d0 (k1): {info['D'][0]}")
    print(f"  cam_d1 (k2): {info['D'][1]}")
    print(f"  cam_d2 (p1): {info['D'][2]}")
    print(f"  cam_d3 (p2): {info['D'][3]}")
    print(f"  cam_d4 (k3): {info['D'][4]}")
else:
    print(f"\n[INTERPRETATION] K appears to be NORMALIZED:")
    print(f"  fx_normalized: {info['K'][0]}")
    print(f"  fy_normalized: {info['K'][4]}")
    print(f"  fx_pixel = {info['K'][0] * info['width']}")
    print(f"  fy_pixel = {info['K'][4] * info['height']}")

# Also check image format
cur.execute("SELECT id FROM topics WHERE name LIKE '%image_raw%'")
img_rows = cur.fetchall()
if img_rows:
    img_topic_id = img_rows[0][0]
    cur.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp LIMIT 1", (img_topic_id,))
    img_row = cur.fetchone()
    if img_row:
        data = bytes(img_row[0])
        pos = 0
        pos = 12  # header
        flen = struct.unpack_from('<I', data, pos)[0]; pos += 4
        frame_id = data[pos:pos+flen-1].decode(errors='replace'); pos += flen
        pos = (pos + 3) // 4 * 4
        mlen = struct.unpack_from('<I', data, pos)[0]; pos += 4
        encoding = data[pos:pos+mlen-1].decode(errors='replace'); pos += mlen
        pos = (pos + 7) // 8 * 8
        ih = struct.unpack_from('<I', data, pos)[0]; pos += 4
        iw = struct.unpack_from('<I', data, pos)[0]
        print(f"\nImage:")
        print(f"  encoding='{encoding}'")
        print(f"  height={ih}, width={iw}")

conn.close()
