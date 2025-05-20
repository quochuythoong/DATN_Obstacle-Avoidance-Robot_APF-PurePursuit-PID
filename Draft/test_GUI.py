import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Original OpenCV-style button positions (x1, y1, x2, y2)
START_BUTTON_POS     = (5, 10, 80, 40)       # Green Start button  
RESET_BUTTON_POS     = (90, 10, 165, 40)     # Red Reset button  
CLEAR_BUTTON_POS     = (90, 50, 165, 80)     # Blue Clear button  
PLAN_PATH_BUTTON_POS = (5, 50, 80, 80)       # Yellow PLAN_PATH button  
APF_PAPF_BUTTON_POS  = (5, 90, 165, 120)     # Green/Red APF_PAPF button  
RUN_BUTTON_POS       = (180, 10, 260, 120)   # Orange RUN button  


# START_BUTTON_POS     = (5, 10, 65, 30)      # Green Start button
# RESET_BUTTON_POS     = (75, 10, 135, 30)    # Red Reset button
# CLEAR_BUTTON_POS     = (75, 35, 135, 55)    # Blue Clear button
# PLAN_PATH_BUTTON_POS = (5, 35, 65, 55)      # Yellow PLAN_PATH button (Initially hidden)
# APF_PAPF_BUTTON_POS  = (5, 60, 135, 80)     # Green/Red APF_PAPF button (toggle APF_PAPF flag)
# RUN_BUTTON_POS       = (145, 10, 205, 80)   # Orange RUN button (appears after path planning)

# Other display parameters
flag_text_x, flag_text_y = 50, 150
flag_invalid_goal_x, flag_invalid_goal_y = 50, 200
frame_height = 600

def cv2_to_matplotlib_rect(pos):
    """Convert OpenCV (x1,y1,x2,y2) to Matplotlib (x,y,width,height)"""
    return (pos[0], pos[1], pos[2]-pos[0], pos[3]-pos[1])

def draw_text_centered(ax, text, cv2_pos, font_size=12, color='black'):
    """Draw text centered in OpenCV-style rectangle"""
    x, y, width, height = cv2_to_matplotlib_rect(cv2_pos)
    ax.text(x + width/2, y + height/2, text, 
            ha='center', va='center', 
            fontsize=font_size, color=color)

def draw_overlay_matplotlib():
    # Create figure
    fig, ax = plt.subplots(figsize=(15, 8))
    ax.set_xlim(0, 1000)
    ax.set_ylim(frame_height, 0)  # Matches OpenCV's coordinate system
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Simulation of your global variables
    coordinates_ready = True
    flag_initialize_direction = True
    run_robot = True
    predictive_APF_enable = True
    flag_goal_inside_obstacle = False
    flag_valid_goal = True
    goal_set_points = [400, 400]  # Example coordinates
    
    # Draw buttons with original OpenCV parameters
    buttons = [
        (START_BUTTON_POS, "START", (0, 255, 0), (0, 0, 0)),  # Green, Black text
        (RESET_BUTTON_POS, "RESET", (0, 0, 255), (255, 255, 255)),  # Red, White text
        (CLEAR_BUTTON_POS, "CLEAR", (255, 0, 0), (255, 255, 255)),  # Blue, White text
    ]
    
    # Conditional buttons
    if coordinates_ready and flag_initialize_direction:
        buttons.append((PLAN_PATH_BUTTON_POS, "PLAN", (0, 255, 255), (0, 0, 0)))  # Yellow, Black text
    
    if run_robot:
        buttons.append((RUN_BUTTON_POS, "RUN", (0, 165, 255), (0, 0, 0)))  # Orange, Black text
    
    # APF-PAPF toggle button
    button_color = (0, 200, 100) if predictive_APF_enable else (200, 50, 50)
    buttons.append((APF_PAPF_BUTTON_POS, "APF-PAPF", button_color, (255, 255, 255)))
    
    # Draw all buttons
    for pos, text, bgr_color, text_color in buttons:
        # Convert BGR to RGB
        facecolor = (bgr_color[2]/255, bgr_color[1]/255, bgr_color[0]/255)
        textcolor = (text_color[2]/255, text_color[1]/255, text_color[0]/255)
        
        rect = patches.Rectangle(
            (pos[0], pos[1]),  # (x,y)
            pos[2]-pos[0],     # width
            pos[3]-pos[1],     # height
            facecolor=facecolor, 
            edgecolor='black',
            linewidth=1
        )
        ax.add_patch(rect)
        draw_text_centered(ax, text, pos, color=textcolor)
    
    # Status text (PAPF/APF)
    status_text = "PAPF Enabled" if predictive_APF_enable else "APF Enabled"
    status_color = (0, 255, 0) if predictive_APF_enable else (0, 0, 255)
    ax.text(flag_text_x, flag_text_y, status_text,
            fontsize=12, 
            color=(status_color[2]/255, status_color[1]/255, status_color[0]/255))
    
    # Goal point (in original OpenCV coordinates)
    if goal_set_points and not flag_goal_inside_obstacle:
        ax.plot(goal_set_points[0], goal_set_points[1], 
               'ro', markersize=10, markeredgecolor='black')
    
    # Warning/status messages
    if flag_goal_inside_obstacle:
        ax.text(flag_invalid_goal_x, flag_invalid_goal_y,
                "Goal is inside obstacle, please Reset & choose a new Goal.",
                fontsize=12, color='red')
    
    if flag_valid_goal:
        ax.text(flag_invalid_goal_x, flag_invalid_goal_y,
                "Goal set point is valid, please proceed.",
                fontsize=12, color=(1, 1, 0))  # Yellow
        ax.text(flag_invalid_goal_x, flag_invalid_goal_y + 30,
                "Or Reset & choose a new Goal.",
                fontsize=12, color=(1, 1, 0))
    
    plt.tight_layout()
    plt.show()

draw_overlay_matplotlib()