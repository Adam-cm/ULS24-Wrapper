import ctypes
import numpy as np
import os
import time

# Load the shared library
so_path = '/home/adam1/ULS24-Wrapper/TestCl/ULSLIB.so'
if not os.path.exists(so_path):
    raise FileNotFoundError(f"Could not find shared library at {so_path}")

ULS24 = ctypes.CDLL(so_path)

# Set argument types
ULS24.selchan.argtypes = [ctypes.c_int]
ULS24.get.argtypes = [ctypes.c_int]
ULS24.setinttime.argtypes = [ctypes.c_float]
ULS24.setgain.argtypes = [ctypes.c_int]
ULS24.reset.argtypes = []
ULS24.get_frame12.argtypes = [ctypes.POINTER(ctypes.c_int)]
ULS24.check_data_flow.argtypes = []
ULS24.check_data_flow.restype = ctypes.c_int

# First diagnostic: check if device responds at all
print("Resetting device...")
result = ULS24.reset()
print(f"Reset result: {result}")

# Initialize the sensor
print("Setting up sensor...")
ULS24.selchan(1)
ULS24.setinttime(4.0)
ULS24.setgain(1)

# Check for any queued data before capture
print("Checking for existing data...")
count = ULS24.check_data_flow()
print(f"Found {count} reports in queue before capture")

# Start capture with a timeout
print("Requesting frame with timeout...")
start_time = time.time()
result = ULS24.get(1)  # This should now timeout if no data arrives
end_time = time.time()
print(f"Frame capture completed in {end_time - start_time:.2f} seconds with result: {result}")

# Check again for any data
count = ULS24.check_data_flow()
print(f"Found {count} reports in queue after capture")

# Try to get the frame anyway
FrameArrayType = ctypes.c_int * (12 * 12)
frame_buffer = FrameArrayType()
ULS24.get_frame12(frame_buffer)
frame_np = np.ctypeslib.as_array(frame_buffer).reshape((12, 12))
print("Frame data (even if incomplete):")
print(frame_np)