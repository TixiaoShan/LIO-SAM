#!env python
# -*- coding: utf-8 -*-

import sys

try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)

import tf
import os
import cv2
import rospy
import rosbag
from tqdm import tqdm
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge
import numpy as np
import argparse

def save_imu_data(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        q = tf.transformations.quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu
        bag.write(topic, imu, t=imu.header.stamp)

def save_imu_data_raw(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU Raw")
    synced_path = kitti.data_path
    unsynced_path = synced_path.replace('sync', 'extract')
    imu_path = os.path.join(unsynced_path, 'oxts')

    # read time stamp (convert to ros seconds format)
    with open(os.path.join(imu_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        imu_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            timestamp = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            imu_datetimes.append(float(timestamp.strftime("%s.%f")))

    # fix imu time using a linear model (may not be ideal, ^_^)
    imu_index = np.asarray(range(len(imu_datetimes)), dtype=np.float64)
    z = np.polyfit(imu_index, imu_datetimes, 1)
    imu_datetimes_new = z[0] * imu_index + z[1]
    imu_datetimes = imu_datetimes_new.tolist()

    # get all imu data
    imu_data_dir = os.path.join(imu_path, 'data')
    imu_filenames = sorted(os.listdir(imu_data_dir))
    imu_data = [None] * len(imu_filenames)
    for i, imu_file in enumerate(imu_filenames):
        imu_data_file = open(os.path.join(imu_data_dir, imu_file), "r")
        for line in imu_data_file:
            if len(line) == 1:
                continue
            stripped_line = line.strip()
            line_list = stripped_line.split()
            imu_data[i] = line_list

    assert len(imu_datetimes) == len(imu_data)
    
    for timestamp, data in zip(imu_datetimes, imu_data):
        roll, pitch, yaw = float(data[3]), float(data[4]), float(data[5]), 
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = rospy.Time.from_sec(timestamp)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = float(data[11])
        imu.linear_acceleration.y = float(data[12])
        imu.linear_acceleration.z = float(data[13])
        imu.angular_velocity.x = float(data[17])
        imu.angular_velocity.y = float(data[18])
        imu.angular_velocity.z = float(data[19])
        bag.write(topic, imu, t=imu.header.stamp)

        imu.header.frame_id = 'imu_enu_link'
        bag.write('/imu_correct', imu, t=imu.header.stamp) # for LIO-SAM GPS

def save_dynamic_tf(bag, kitti, kitti_type, initial_time):
    print("Exporting time dependent transformations")
    if kitti_type.find("raw") != -1:
        for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
            tf_oxts_msg = TFMessage()
            tf_oxts_transform = TransformStamped()
            tf_oxts_transform.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
            tf_oxts_transform.header.frame_id = 'world'
            tf_oxts_transform.child_frame_id = 'base_link'

            transform = (oxts.T_w_imu)
            t = transform[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(transform)
            oxts_tf = Transform()

            oxts_tf.translation.x = t[0]
            oxts_tf.translation.y = t[1]
            oxts_tf.translation.z = t[2]

            oxts_tf.rotation.x = q[0]
            oxts_tf.rotation.y = q[1]
            oxts_tf.rotation.z = q[2]
            oxts_tf.rotation.w = q[3]

            tf_oxts_transform.transform = oxts_tf
            tf_oxts_msg.transforms.append(tf_oxts_transform)

            bag.write('/tf', tf_oxts_msg, tf_oxts_msg.transforms[0].header.stamp)

    elif kitti_type.find("odom") != -1:
        timestamps = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
        for timestamp, tf_matrix in zip(timestamps, kitti.T_w_cam0):
            tf_msg = TFMessage()
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = rospy.Time.from_sec(timestamp)
            tf_stamped.header.frame_id = 'world'
            tf_stamped.child_frame_id = 'camera_left'
            
            t = tf_matrix[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(tf_matrix)
            transform = Transform()

            transform.translation.x = t[0]
            transform.translation.y = t[1]
            transform.translation.z = t[2]

            transform.rotation.x = q[0]
            transform.rotation.y = q[1]
            transform.rotation.z = q[2]
            transform.rotation.w = q[3]

            tf_stamped.transform = transform
            tf_msg.transforms.append(tf_stamped)

            bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)

def save_camera_data(bag, kitti_type, kitti, util, bridge, camera, camera_frame_id, topic, initial_time):
    print("Exporting camera {}".format(camera))
    if kitti_type.find("raw") != -1:
        camera_pad = '{0:02d}'.format(camera)
        image_dir = os.path.join(kitti.data_path, 'image_{}'.format(camera_pad))
        image_path = os.path.join(image_dir, 'data')
        image_filenames = sorted(os.listdir(image_path))
        with open(os.path.join(image_dir, 'timestamps.txt')) as f:
            image_datetimes = map(lambda x: datetime.strptime(x[:-4], '%Y-%m-%d %H:%M:%S.%f'), f.readlines())
        
        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.width, calib.height = tuple(util['S_rect_{}'.format(camera_pad)].tolist())
        calib.distortion_model = 'plumb_bob'
        calib.K = util['K_{}'.format(camera_pad)]
        calib.R = util['R_rect_{}'.format(camera_pad)]
        calib.D = util['D_{}'.format(camera_pad)]
        calib.P = util['P_rect_{}'.format(camera_pad)]
            
    elif kitti_type.find("odom") != -1:
        camera_pad = '{0:01d}'.format(camera)
        image_path = os.path.join(kitti.sequence_path, 'image_{}'.format(camera_pad))
        image_filenames = sorted(os.listdir(image_path))
        image_datetimes = map(lambda x: initial_time + x.total_seconds(), kitti.timestamps)
        
        calib = CameraInfo()
        calib.header.frame_id = camera_frame_id
        calib.P = util['P{}'.format(camera_pad)]
    
    iterable = zip(image_datetimes, image_filenames)
    for dt, filename in tqdm(iterable, total=len(image_filenames)):
        image_filename = os.path.join(image_path, filename)
        cv_image = cv2.imread(image_filename)
        calib.height, calib.width = cv_image.shape[:2]
        if camera in (0, 1):
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        encoding = "mono8" if camera in (0, 1) else "bgr8"
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        image_message.header.frame_id = camera_frame_id
        if kitti_type.find("raw") != -1:
            image_message.header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))
            topic_ext = "/image_raw"
        elif kitti_type.find("odom") != -1:
            image_message.header.stamp = rospy.Time.from_sec(dt)
            topic_ext = "/image_rect"
        calib.header.stamp = image_message.header.stamp
        bag.write(topic + topic_ext, image_message, t = image_message.header.stamp)
        bag.write(topic + '/camera_info', calib, t = calib.header.stamp) 
        
def save_velo_data(bag, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_data_dir = os.path.join(velo_path, 'data')
    velo_filenames = sorted(os.listdir(velo_data_dir))
    with open(os.path.join(velo_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            velo_datetimes.append(dt)

    iterable = zip(velo_datetimes, velo_filenames)

    count = 0

    for dt, filename in tqdm(iterable, total=len(velo_filenames)):
        if dt is None:
            continue

        velo_filename = os.path.join(velo_data_dir, filename)

        # read binary data
        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

        # get ring channel
        depth = np.linalg.norm(scan, 2, axis=1)
        pitch = np.arcsin(scan[:, 2] / depth) # arcsin(z, depth)
        fov_down = -24.8 / 180.0 * np.pi
        fov = (abs(-24.8) + abs(2.0)) / 180.0 * np.pi
        proj_y = (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]
        proj_y *= 64  # in [0.0, H]
        proj_y = np.floor(proj_y)
        proj_y = np.minimum(64 - 1, proj_y)
        proj_y = np.maximum(0, proj_y).astype(np.int32)  # in [0,H-1]
        proj_y = proj_y.reshape(-1, 1)
        scan = np.concatenate((scan,proj_y), axis=1)
        scan = scan.tolist()
        for i in range(len(scan)):
            scan[i][-1] = int(scan[i][-1])

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))

        # fill pcl msg
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1),
                  PointField('ring', 16, PointField.UINT16, 1)]
        pcl_msg = pcl2.create_cloud(header, fields, scan)
        pcl_msg.is_dense = True
        # print(pcl_msg)

        bag.write(topic, pcl_msg, t=pcl_msg.header.stamp)

        # count += 1
        # if count > 200:
        #     break

def get_static_transform(from_frame_id, to_frame_id, transform):
    t = transform[0:3, 3]
    q = tf.transformations.quaternion_from_matrix(transform)
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg


def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def save_static_transforms(bag, transforms, timestamps):
    print("Exporting static transformations")
    tfm = TFMessage()
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])
        tfm.transforms.append(t)
    for timestamp in timestamps:
        time = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time
        bag.write('/tf_static', tfm, t=time)


def save_gps_fix_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        navsatfix_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        bag.write(topic, navsatfix_msg, t=navsatfix_msg.header.stamp)


def save_gps_vel_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        twist_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        bag.write(topic, twist_msg, t=twist_msg.header.stamp)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Convert KITTI dataset to ROS bag file the easy way!")
    # Accepted argument values
    kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    odometry_sequences = []
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2))
    
    parser.add_argument("kitti_type", choices = kitti_types, help = "KITTI dataset type")
    parser.add_argument("dir", nargs = "?", default = os.getcwd(), help = "base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("-t", "--date", help = "date of the raw dataset (i.e. 2011_09_26), option is only for RAW datasets.")
    parser.add_argument("-r", "--drive", help = "drive number of the raw dataset (i.e. 0001), option is only for RAW datasets.")
    parser.add_argument("-s", "--sequence", choices = odometry_sequences,help = "sequence of the odometry dataset (between 00 - 21), option is only for ODOMETRY datasets.")
    args = parser.parse_args()

    bridge = CvBridge()
    compression = rosbag.Compression.NONE
    # compression = rosbag.Compression.BZ2
    # compression = rosbag.Compression.LZ4
    
    # CAMERAS
    cameras = [
        (0, 'camera_gray_left', '/kitti/camera_gray_left'),
        (1, 'camera_gray_right', '/kitti/camera_gray_right'),
        (2, 'camera_color_left', '/kitti/camera_color_left'),
        (3, 'camera_color_right', '/kitti/camera_color_right')
    ]

    if args.kitti_type.find("raw") != -1:
    
        if args.date == None:
            print("Date option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)
        elif args.drive == None:
            print("Drive option is not given. It is mandatory for raw dataset.")
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -t <date> -r <drive>")
            sys.exit(1)
        
        bag = rosbag.Bag("kitti_{}_drive_{}_{}.bag".format(args.date, args.drive, args.kitti_type[4:]), 'w', compression=compression)
        kitti = pykitti.raw(args.dir, args.date, args.drive)
        if not os.path.exists(kitti.data_path):
            print('Path {} does not exists. Exiting.'.format(kitti.data_path))
            sys.exit(1)

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        try:
            # IMU
            imu_frame_id = 'imu_link'
            imu_topic = '/kitti/oxts/imu'
            imu_raw_topic = '/imu_raw'
            gps_fix_topic = '/gps/fix'
            gps_vel_topic = '/gps/vel'
            velo_frame_id = 'velodyne'
            velo_topic = '/points_raw'

            T_base_link_to_imu = np.eye(4, 4)
            T_base_link_to_imu[0:3, 3] = [-2.71/2.0-0.05, 0.32, 0.93]

            # tf_static
            transforms = [
                ('base_link', imu_frame_id, T_base_link_to_imu),
                (imu_frame_id, velo_frame_id, inv(kitti.calib.T_velo_imu)),
                (imu_frame_id, cameras[0][1], inv(kitti.calib.T_cam0_imu)),
                (imu_frame_id, cameras[1][1], inv(kitti.calib.T_cam1_imu)),
                (imu_frame_id, cameras[2][1], inv(kitti.calib.T_cam2_imu)),
                (imu_frame_id, cameras[3][1], inv(kitti.calib.T_cam3_imu))
            ]

            util = pykitti.utils.read_calib_file(os.path.join(kitti.calib_path, 'calib_cam_to_cam.txt'))

            # Export
            # save_static_transforms(bag, transforms, kitti.timestamps)
            # save_dynamic_tf(bag, kitti, args.kitti_type, initial_time=None)
            # save_imu_data(bag, kitti, imu_frame_id, imu_topic)
            save_imu_data_raw(bag, kitti, imu_frame_id, imu_raw_topic)
            save_gps_fix_data(bag, kitti, imu_frame_id, gps_fix_topic)
            save_gps_vel_data(bag, kitti, imu_frame_id, gps_vel_topic)
            for camera in cameras:
                save_camera_data(bag, args.kitti_type, kitti, util, bridge, camera=camera[0], camera_frame_id=camera[1], topic=camera[2], initial_time=None)
                break
            save_velo_data(bag, kitti, velo_frame_id, velo_topic)

        finally:
            print("## OVERVIEW ##")
            print(bag)
            bag.close()
            
    elif args.kitti_type.find("odom") != -1:
        
        if args.sequence == None:
            print("Sequence option is not given. It is mandatory for odometry dataset.")
            print("Usage for odometry dataset: kitti2bag {odom_color, odom_gray} [dir] -s <sequence>")
            sys.exit(1)
            
        bag = rosbag.Bag("kitti_data_odometry_{}_sequence_{}.bag".format(args.kitti_type[5:],args.sequence), 'w', compression=compression)
        
        kitti = pykitti.odometry(args.dir, args.sequence)
        if not os.path.exists(kitti.sequence_path):
            print('Path {} does not exists. Exiting.'.format(kitti.sequence_path))
            sys.exit(1)

        kitti.load_calib()         
        kitti.load_timestamps() 
             
        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)
            
        if args.sequence in odometry_sequences[:11]:
            print("Odometry dataset sequence {} has ground truth information (poses).".format(args.sequence))
            kitti.load_poses()

        try:
            util = pykitti.utils.read_calib_file(os.path.join(args.dir,'sequences',args.sequence, 'calib.txt'))
            current_epoch = (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
            # Export
            if args.kitti_type.find("gray") != -1:
                used_cameras = cameras[:2]
            elif args.kitti_type.find("color") != -1:
                used_cameras = cameras[-2:]

            save_dynamic_tf(bag, kitti, args.kitti_type, initial_time=current_epoch)
            for camera in used_cameras:
                save_camera_data(bag, args.kitti_type, kitti, util, bridge, camera=camera[0], camera_frame_id=camera[1], topic=camera[2], initial_time=current_epoch)

        finally:
            print("## OVERVIEW ##")
            print(bag)
            bag.close()

