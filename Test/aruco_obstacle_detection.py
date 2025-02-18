import cv2
import numpy as np
from cv2 import aruco

def detect_aruco_and_obstacles():
    cap = cv2.VideoCapture(0)  # Open camera
    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  # Define ArUco dictionary
    aruco_params = aruco.DetectorParameters_create()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        
        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        aruco_coords = None
        
        if ids is not None:
            for i, corner in enumerate(corners):
                corner = corner[0]
                aruco_coords = tuple(np.mean(corner, axis=0))  # Get ArUco marker center
                cv2.polylines(frame, [corner.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.putText(frame, f"ID: {ids[i][0]}", tuple(corner[0].astype(int)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Detect obstacles (anything not ArUco marker)
        mask = np.zeros_like(gray)
        if ids is not None:
            for corner in corners:
                cv2.fillPoly(mask, [corner.astype(int)], 255)
        
        obstacles = cv2.bitwise_not(mask)
        contours, _ = cv2.findContours(obstacles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obstacle_boxes = []
        
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w * h > 500:  # Filter out small noise
                obstacle_boxes.append((x, y, x + w, y + h))
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                
        # Display the results
        cv2.imshow('Aruco and Obstacles Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return aruco_coords, obstacle_boxes

if __name__ == "__main__":
    aruco_coords, obstacle_boxes = detect_aruco_and_obstacles()
    print(f"Aruco Marker Coordinates: {aruco_coords}")
    print(f"Obstacle Bounding Boxes: {obstacle_boxes}")
