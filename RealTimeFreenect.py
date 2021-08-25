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
    vis.create_window()
    pinholeCamera = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    colorsListener = device.color_frame_listener
    device.start()
    pcdList = []

    while True:
        img_depth, img_rgb = freeCam(device)
        depth = img_depth.to_array()
        rgb = img_rgb.to_array()
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGBA2BGR)

        depth = np.delete(depth, -1, 0)
        depth = np.delete(depth, 0, 0)
        image_depth = o3d.geometry.Image(depth)
        image_rgb = o3d.geometry.Image(rgb)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image_rgb, image_depth, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinholeCamera)
        pcdList.append(pcd)
        vis.add_geometry(pcdList[-1])
        if len(pcdList) > 1:
            vis.remove_geometry(pcdList[0])
        vis.poll_events()
        vis.update_renderer()


