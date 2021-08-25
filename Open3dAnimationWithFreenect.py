from freenect2 import Device, FrameType
import numpy as np
import open3d as o3d
import cv2
import queue
import time

def main():
    try:
        device = Device()

        vis = o3d.visualization.Visualizer()
        pcd = o3d.geometry.PointCloud()
        counter = 0
        vis.create_window()
        frames = {}
        pinholeCamera = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        with device.running():
            for type_, frame in device:
                print(counter)
                counter += 1
                frames[type_] = frame
                if FrameType.Color in frames and FrameType.Depth in frames:
                    rgb, depth = frames[FrameType.Color], frames[FrameType.Depth]
                    print("2")
                    undistorted, registered = device.registration.apply(rgb, depth, enable_filter=True)
                    print("3")
                    undistorted = undistorted.to_array()
                    registered = registered.to_array()
                    registered = cv2.cvtColor(registered, cv2.COLOR_RGBA2BGR)
                    image_depth = o3d.geometry.Image(undistorted)
                    image_rgb = o3d.geometry.Image(registered)
                    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image_rgb, image_depth, convert_rgb_to_intensity=False)
                    pcd.clear()
                    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinholeCamera)
                    vis.clear_geometries()
                    vis.add_geometry(pcd)
                    vis.poll_events()
                    vis.update_renderer()
                    continue
            vis.destroy_window()
    except:
        print("except::")
    finally:
        main()


if __name__ == '__main__':
    main()