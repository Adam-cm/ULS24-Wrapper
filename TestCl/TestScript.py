import ctypes
import numpy as np
import os
import time
import atexit
import subprocess

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
ULS24.cancel_capture.argtypes = []
ULS24.optimize_for_pi.argtypes = []

# Try to optimize the system
try:
    # Ensure no other USB devices are using bandwidth
    subprocess.run("echo 'minimize USB usage' | wall", shell=True)
    
    # Try to optimize the process
    ULS24.optimize_for_pi()
    
    # Set this process to high priority
    os.nice(-20)
except Exception as e:
    print(f"Could not apply all optimizations: {e}")

# Ensure we always clean up
def cleanup():
    ULS24.cancel_capture()
    time.sleep(0.5)  # Give time for cleanup

atexit.register(cleanup)

try:
    # Reset and initialize
    print("Resetting device...")
    result = ULS24.reset()
    print(f"Reset result: {result}")

    print("Setting up sensor...")
    ULS24.selchan(1)
    ULS24.setinttime(4.0)
    ULS24.setgain(1)
    
    # Clear any pending data
    count = ULS24.check_data_flow()
    if count > 0:
        print(f"Cleared {count} pending reports")

    # Capture frame
    print("Requesting frame...")
    start_time = time.time()
    result = ULS24.get(1)
    end_time = time.time()
    print(f"Frame capture completed in {end_time - start_time:.2f} seconds with result: {result}")
    
    # Get frame data
    FrameArrayType = ctypes.c_int * (12 * 12)
    frame_buffer = FrameArrayType()
    ULS24.get_frame12(frame_buffer)
    frame_np = np.ctypeslib.as_array(frame_buffer).reshape((12, 12))
    
    print("\nFrame data:")
    print(frame_np)

except Exception as e:
    print(f"Error: {e}")

print("Test complete")