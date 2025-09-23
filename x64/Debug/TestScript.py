import ctypes
import numpy as np
import os

# Load the .so shared library (update the path as needed)
# Example: if your .so is in /home/pi/ULS24-Wrapper/TestCl/libTestCl.so
so_path = '/home/pi/ULS24-Wrapper/TestCl/ULSLIB.so'
if not os.path.exists(so_path):
    raise FileNotFoundError(f"Could not find shared library at {so_path}")

ULS24 = ctypes.CDLL(so_path)

# Set argument types if needed
ULS24.selchan.argtypes = [ctypes.c_int]
ULS24.get.argtypes = [ctypes.c_int]
ULS24.setinttime.argtypes = [ctypes.c_float]
ULS24.setgain.argtypes = [ctypes.c_int]
ULS24.reset.argtypes = []
ULS24.get_frame12.argtypes = [ctypes.POINTER(ctypes.c_int)]

# Call the functions
print("Calling reset")
ULS24.reset()
print("Calling selchan")
ULS24.selchan(1)
print("Calling setinttime")
ULS24.setinttime(4.0)
print("Calling setgain")
ULS24.setgain(1)
print("Calling get")
ULS24.get(1)
print("Done")

# After calling get(chan), retrieve the frame data
FrameArrayType = ctypes.c_int * (12 * 12)
frame_buffer = FrameArrayType()
ULS24.get_frame12(frame_buffer)
frame_np = np.ctypeslib.as_array(frame_buffer).reshape((12, 12))
print(frame_np)