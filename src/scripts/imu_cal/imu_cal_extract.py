#!/usr/bin/env python3
"""Dump the topics needed for the IMU calibration from the bag to npz."""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, out = sys.argv[1], sys.argv[2]
want = ['/imu/data_raw', '/imu_cal/phase', '/scan_filtered', '/diff_cont/odom', '/odom_rf2o', '/rosout', '/cmd_vel']
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
cls = {t: get_message(types[t]) for t in want if t in types}
imu, ph, odom, rf2o, logs, cmd = [], [], [], [], [], []; scans = []
while r.has_next():
    t, data, bt = r.read_next(); m = deserialize_message(data, cls[t]); bt *= 1e-9
    if t == '/imu/data_raw':
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        imu.append((bt, st, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    elif t == '/imu_cal/phase': ph.append((bt, m.data))
    elif t == '/diff_cont/odom': odom.append((bt, m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif t == '/odom_rf2o': rf2o.append((bt, m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif t == '/cmd_vel': cmd.append((bt, m.linear.x, m.angular.z))
    elif t == '/rosout': logs.append((bt, m.name, m.level, m.msg))
    elif t == '/scan_filtered':
        scans.append((bt, m.angle_min, m.angle_increment, np.array(m.ranges, dtype=np.float32), m.range_min, m.range_max))
np.savez(out, imu=np.array(imu), odom=np.array(odom), rf2o=np.array(rf2o), cmd=np.array(cmd),
         ph_t=np.array([p[0] for p in ph]), ph_l=np.array([p[1] for p in ph]),
         log_t=np.array([l[0] for l in logs]), log_n=np.array([l[1] for l in logs]), log_m=np.array([l[3] for l in logs]),
         scan_t=np.array([s[0] for s in scans]), scan_a0=np.array([s[1] for s in scans]), scan_da=np.array([s[2] for s in scans]),
         scan_r=np.array([s[3] for s in scans]))
print('imu', len(imu), 'phases', ph, 'scans', len(scans), 'odom', len(odom), 'rf2o', len(rf2o), 'logs', len(logs))
