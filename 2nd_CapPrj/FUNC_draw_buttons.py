import cv2

def draw_buttons(frame, start_button_pos, reset_button_pos, button_width, button_height):
    cv2.rectangle(frame, start_button_pos, (start_button_pos[0] + button_width, start_button_pos[1] + button_height), (0, 255, 0), -1)
    cv2.putText(frame, "Start", (start_button_pos[0] + 15, start_button_pos[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.rectangle(frame, reset_button_pos, (reset_button_pos[0] + button_width, reset_button_pos[1] + button_height), (0, 0, 255), -1)
    cv2.putText(frame, "Reset", (reset_button_pos[0] + 15, reset_button_pos[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
