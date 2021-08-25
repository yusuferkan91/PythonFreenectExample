import pyrealsense2 as rs
import cv2
import numpy as np
import open3d as o3d

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

align_to = rs.stream.color
align = rs.align(align_to)

profile = pipeline.start(config)


depth_sensor = profile.get_device().first_depth_sensor()

depth_scale = depth_sensor.get_depth_scale()

colorizer = rs.colorizer()
vis = o3d.visualization.Visualizer()
vis.create_window("3D Map")
pinholeCamera = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
pcd = o3d.io.read_point_cloud("B_tup_1.pcd")
vis.add_geometry(pcd)
vis.poll_events()
vis.update_renderer()
try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)


        image_depth = o3d.geometry.Image(depth_image)
        image_rgb = o3d.geometry.Image(color_image)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image_rgb, image_depth, depth_scale=5000, convert_rgb_to_intensity=False)

        temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinholeCamera)

        pcd.points = temp.points
        pcd.colors = temp.colors
        pcd.normals = temp.normals
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

finally:
    pipeline.stop()