import cv2
import numpy as np

def process_frame(cap, gamma=500):
    """Captures and processes a frame from the camera with reduced exposure"""
    ret, frame = cap.read()
    if not ret:
        return None, None
    
    # Apply gamma correction to reduce exposure (values > 1 darken the image)
    inv_gamma = 1.0 / gamma
    lookup_table = np.array([((i / 255.0) ** inv_gamma) * 255 
                           for i in np.arange(0, 256)]).astype("uint8")
    adjusted_frame = cv2.LUT(frame, lookup_table)
    
    # Convert to grayscale after exposure adjustment
    gray = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2GRAY)
    
    return adjusted_frame, gray

cap = cv2.VideoCapture(0)

while True:
    frame, gray = process_frame(cap, gamma=1.8)  # Higher gamma = darker
    
    if frame is None:
        break
        
    cv2.imshow('Adjusted Color', frame)
    cv2.imshow('Adjusted Grayscale', gray)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()