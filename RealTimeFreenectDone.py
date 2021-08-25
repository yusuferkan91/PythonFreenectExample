from freenect2 import Device, FrameType, Queue, frame_listener_callback, QueueFrameListener
import cv2
import open3d as o3d
import numpy as np

def freeCam(dev):
    frames = {}
    for type_, frame in device:
        frames[type_] = frame
        if FrameType.Depth in frames and FrameType.Color in frames:
            depth, rgb = frames[FrameType.Depth], frames[FrameType.Color]
            undistorted, registered, big_depth = device.registration.apply(rgb, depth, with_big_depth=True)
            return big_depth, rgb

if __name__ == "__main__":
    device = Device()
    vis = o3d.visualization.Visualizer()
    vis.create_window("3D Map")
    pinholeCamera = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    colorsListener = device.color_frame_listener
    device.start()
    device.stop()
    device.start()
    count = 0
    pcd = o3d.io.read_point_cloud("B_tup_2.pcd")
    vis.add_geometry(pcd)
    queList = {}
    print(colorsListener.queue.qsize())
    while True:
        x = colorsListener.queue.get()
        if x[0] == FrameType.Color:
            queList[FrameType.Color] = x[1]
        if x[0] == FrameType.Depth:
            queList[FrameType.Depth] = x[1]
        if len(queList) == 2:
            undistorted, registered = device.registration.apply(queList[FrameType.Color], queList[FrameType.Depth])
            rgb = registered.to_array()
            depth = queList[FrameType.Depth].to_array()
            """---------FILTER-----------"""
            depth[depth > 1000] = None
            depth[:80] = None
            """--------------------------"""
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGBA2BGR)
            print(rgb.shape, depth.shape)
            image_depth = o3d.geometry.Image(depth)
            image_rgb = o3d.geometry.Image(rgb)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image_rgb, image_depth, convert_rgb_to_intensity=False)
            temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinholeCamera)
            pcd.points = temp.points
            pcd.colors = temp.colors
            pcd.normals = temp.normals
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()
            queList.clear()
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
    device.stop()



