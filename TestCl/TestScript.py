import ctypes
import numpy as np
import os
import time
import atexit
import platform
import sys

# Determine the path of the shared library based on platform
if platform.system() == 'Linux':
    lib_name = 'ULSLIB.so'
else:  # Windows
    lib_name = 'ULSLIB.dll'

# Try to find the library - check multiple locations
script_dir = os.path.dirname(os.path.abspath(__file__))
possible_paths = [
    os.path.join(script_dir, lib_name),
    os.path.join(script_dir, '..', lib_name),
    os.path.join(os.path.dirname(script_dir), lib_name)
]

# Add user-provided path if running on Linux
if platform.system() == 'Linux':
    possible_paths.append('/home/adam1/ULS24-Wrapper/TestCl/ULSLIB.so')

# Find the first valid path
so_path = None
for path in possible_paths:
    if os.path.exists(path):
        so_path = path
        break

if not so_path:
    print(f"ERROR: Could not find shared library ({lib_name})")
    print(f"Searched in: {possible_paths}")
    sys.exit(1)

print(f"Loading library from: {so_path}")
ULS24 = ctypes.CDLL(so_path)

# Set argument and return types
ULS24.selchan.argtypes = [ctypes.c_int]
ULS24.get.argtypes = [ctypes.c_int]
ULS24.setinttime.argtypes = [ctypes.c_float]
ULS24.setgain.argtypes = [ctypes.c_int]
ULS24.get_frame12.argtypes = [ctypes.POINTER(ctypes.c_int)]
ULS24.check_data_flow.argtypes = []
ULS24.check_data_flow.restype = ctypes.c_int
ULS24.cancel_capture.argtypes = []

# Ensure we always clean up
def cleanup():
    try:
        ULS24.cancel_capture()
        print("Cleanup complete")
    except:
        pass

atexit.register(cleanup)

# Apply Linux-specific optimizations
if platform.system() == 'Linux':
    try:
        ULS24.optimize_for_pi()
        print("Applied Linux/Pi optimizations")
    except Exception as e:
        print(f"Note: Could not apply all optimizations: {e}")

try:
    # Reset and initialize
    print("Resetting device...")
    ULS24.reset()
    time.sleep(0.5)  # Give device time to initialize
    
    print("Setting up sensor...")
    ULS24.selchan(1)  # Select channel 1
    time.sleep(0.1)
    
    ULS24.setinttime(4.0)  # Set integration time to 4ms
    time.sleep(0.1)
    
    ULS24.setgain(1)  # Set gain mode (1 = high gain)
    time.sleep(0.1)
    
    # Clear any pending data
    count = ULS24.check_data_flow()
    if count > 0:
        print(f"Cleared {count} pending reports")
        
    # Allow more time for device to stabilize
    print("Waiting for device to stabilize...")
    time.sleep(1.0)
    
    # Capture frame with retry capability
    print("\nCapturing frame (this may take a moment)...")
    start_time = time.time()
    
    # Capture frame
    ULS24.get(1)
    
    end_time = time.time()
    print(f"Frame capture completed in {end_time - start_time:.2f} seconds")
    
    # Get frame data
    print("Retrieving frame data...")
    FrameArrayType = ctypes.c_int * (12 * 12)
    frame_buffer = FrameArrayType()
    ULS24.get_frame12(frame_buffer)
    
    # Convert to numpy array for analysis
    frame_np = np.ctypeslib.as_array(frame_buffer).reshape((12, 12))
    
    # Analyze the frame
    non_zero = np.count_nonzero(frame_np)
    total_elements = frame_np.size
    print(f"\nFrame statistics:")
    print(f"- Non-zero values: {non_zero}/{total_elements} ({non_zero/total_elements*100:.1f}%)")
    print(f"- Min value: {np.min(frame_np)}")
    print(f"- Max value: {np.max(frame_np)}")
    print(f"- Mean value: {np.mean(frame_np):.2f}")
    
    print("\nFrame data:")
    print(frame_np)
    
    # Check for missing rows (all zeros)
    zero_rows = []
    for i in range(12):
        if np.count_nonzero(frame_np[i]) == 0:
            zero_rows.append(i)
            
    if zero_rows:
        print(f"\nWarning: Found {len(zero_rows)} empty rows: {zero_rows}")
    else:
        print("\nSuccess: All rows contain data!")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

print("\nTest complete")