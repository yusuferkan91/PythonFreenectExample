from freenect2 import Device, FrameType
import numpy as np
import open3d as o3d
import cv2
def main():
    device = Device()
    frames = {}
    with device.running():
        for type_, frame in device:
            frames[type_] = frame
            if FrameType.Color in frames and FrameType.Depth in frames:
                break

    rgb, depth = frames[FrameType.Color], frames[FrameType.Depth]
    undistorted, registered, big_depth = device.registration.apply(rgb, depth, with_big_depth=True)

    big_depth = big_depth.to_array()
    rgb = rgb.to_array()
    rgb = cv2.cvtColor(rgb,cv2.COLOR_RGBA2RGB)
    big_depth = big_depth[:rgb.shape[0],:]

    image_depth = o3d.geometry.Image(big_depth)
    image_rgb = o3d.geometry.Image(rgb)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image_rgb,image_depth, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd,o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    o3d.visualization.draw_geometries([pcd])
    
if __name__ == '__main__':
    main()