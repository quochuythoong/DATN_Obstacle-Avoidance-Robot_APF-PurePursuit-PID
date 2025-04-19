# Frame dimensions
frame_height = 1080  
frame_width = 1920

# Buttons positions and dimensions
start_button_pos = (10, 30)
reset_button_pos = (10, 70)
button_width = 80
button_height = 30

# Parameters for Pure Pursuit
ConstVelocity = 0.08  # Constant velocity in m/s
LookAHead_dist = 45   # Look-ahead distance in pixels
LookAHead_dist_RealLife = LookAHead_dist * 0.00123292 # Look-ahead distance in meters
Wheels_dist = 0.045    # Distance between wheels in meters