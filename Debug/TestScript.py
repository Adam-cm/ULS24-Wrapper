import ctypes
import numpy as np

# Load the DLL (adjust the path as needed)
ULS24 = ctypes.CDLL(r'C:\Users\Adam1\source\repos\TestCI\x64\Debug\TestCl.dll')

# Set argument types if needed
ULS24.selchan.argtypes = [ctypes.c_int]
ULS24.get.argtypes = [ctypes.c_int]
ULS24.setinttime.argtypes = [ctypes.c_float]
ULS24.setgain.argtypes = [ctypes.c_int]
ULS24.reset.argtypes = []

# Call the functions
print("Calling reset")
ULS24.reset()
print("Calling selchan")
ULS24.selchan(1)
print("Calling setinttime")
ULS24.setinttime(30.0)
print("Calling setgain")
ULS24.setgain(1)
print("Calling get")
ULS24.get(1)
print("Done")

# After calling dll.get(chan), retrieve the frame data
FrameArrayType = ctypes.c_float * (12 * 12)
frame_buffer = FrameArrayType()
ULS24.get_frame12.argtypes = [ctypes.POINTER(ctypes.c_float)]
ULS24.get_frame12.restype = None

ULS24.get_frame12(frame_buffer)
frame_np = np.ctypeslib.as_array(frame_buffer).reshape((12, 12))
print(frame_np)