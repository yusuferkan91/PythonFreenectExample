from freenect2 import Device, FrameType, Registration, Frame, QueueFrameListener
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import get_test_data

# We use numpy to process the raw IR frame
import numpy as np

# We use the Pillow library for saving the captured image
from PIL import Image


# Open default device
device = Device()
# Start the device


with device.running():
    #print("device::",device.shape)
    # For each received frame...
    for type_, frame in device:
        #ir_image = frame.to_array()
        d_device = device["color"]
        if type_ is FrameType.Depth:
            depth_image = frame
        if type_ is FrameType.Color:
            color_image = frame
            break
#registration = Registration(device.getIrCameraParams(),
  #                          device.getColorCameraParams())
#print(depth_image.shape)
big_depht = device.registration.apply(color_image, depth_image)
x,y,z = device.registration.get_points_xyz_array(big_depht)

#ir_image /= ir_image.max()
#ir_image = np.sqrt(ir_image)
#print(depth_image.shape)
#x, y, z  = register.get_points_xyz_array(undistorted=depth_image)
x = np.array(list(range(0,depth_image.shape[0])))
y = np.array(list(range(0,depth_image.shape[1])))

fig = plt.figure()
#plt.plot(depth_image)
ax = fig.add_subplot(1, 2, 2, projection='3d')
ax.plot_wireframe(x, y, depth_image, rstride=10, cstride=10)

plt.show()
#plt.imshow(depth_image)
#plt.imshow(color_image)
#plt.imshow(ir_image)


