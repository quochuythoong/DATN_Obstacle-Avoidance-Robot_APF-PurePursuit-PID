import cv2

def mouse_callback(event, x, y, flags, param):
    clicked_points, reset_button_pos, start_button_pos, button_width, button_height, frame_height, start_pressed, interpolated_waypoints, latest_waypoint, flag_clicked_point_added = param
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # Check if the click was on the reset button using the original y
        if reset_button_pos[0] <= x <= reset_button_pos[0] + button_width and reset_button_pos[1] <= y <= reset_button_pos[1] + button_height:
            clicked_points.clear()
            start_pressed[0] = False  # Reset start_pressed flag
            latest_waypoint = None
            print("Reset button clicked. All points cleared and start flag reset.")
        
        # Check if the click was on the start button
        elif start_button_pos[0] <= x <= start_button_pos[0] + button_width and start_button_pos[1] <= y <= start_button_pos[1] + button_height:
            start_pressed[0] = True  # Set start_pressed flag
            print("Start button clicked. Processing waypoints.")
        
        else:
            # Invert y-coordinate for the clicked point before adding to list
            y = frame_height - y
            clicked_points.append((x, y))
            print(f"Mouse clicked at: X = {x}, Y = {y}")


