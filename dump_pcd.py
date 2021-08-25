from freenect2 import Device, FrameType
import numpy as np
from open3d import io, visualization, geometry, utility, registration

device = Device()
frames = {}
with device.running():
    for type_, frame in device:
        frames[type_] = frame
        if FrameType.Color in frames and FrameType.Depth in frames:
            break

rgb, depth = frames[FrameType.Color], frames[FrameType.Depth]
undistorted, registered, big_depth = device.registration.apply(rgb, depth, with_big_depth=True)
    
with open('output.pcd', 'wb') as fobj:
    device.registration.write_pcd(fobj, undistorted, registered)

with open('output_big.pcd', 'wb') as fobj:
   device.registration.write_big_pcd(fobj, big_depth, rgb)

cloud = io.read_point_cloud("output_big.pcd")
visualization.draw_geometries([cloud])

