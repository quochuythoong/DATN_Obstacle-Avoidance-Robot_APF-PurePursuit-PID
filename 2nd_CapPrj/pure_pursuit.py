import math
from utils import ConstVelocity

k1 = 80 / ConstVelocity # depend on v_robot
k2 = - 60 / (2*ConstVelocity / 0.0215) # 10 = 45 / 4.5 (depend on 2*omega=w1 + w2)
min_ld = 15 
wheel_scale = 0.01075 # R / 2 = 0.0215 / 2 = 0.01075

def calculate_omega(AH, v, lt):
    omega = (2 * AH * v) / (lt ** 2)
    # print(f"Omega: {omega}")
    return omega

def calculate_wheel_velocities(omega, R, Ld):
    v1 = omega * (R + Ld)
    v2 = omega * (R - Ld)
    w1 = v1 / 0.0215 # rad/s
    w2 = v2 / 0.0215
    print(f"Wheel 1: {w1}, Wheel 2: {w2}")
    return w1, w2

def velocities_to_RPM(v1, v2):
    rpm1 = (v1 * 60) / (2 * math.pi * 0.0215) #radius of wheels = 0.0215 meter
    rpm2 = (v2 * 60) / (2 * math.pi * 0.0215)
    PWM1 = (rpm1 / 250) * 255 + 5  #250 max RPM of motor, 255 max PWM of ESP, 5 random calib
    PWM2 = (rpm2 / 250) * 255 + 5
    return PWM1, PWM2

def adaptive_lookahead(w1, w2, omega):
    global k1, k2, min_ld
    v_robot = wheel_scale * (w1 + w2) 
    # print(f"Current velocity: {v_current}")
    # Tuned lookahead distance
    if omega < 0: # Negative omega --> turn right
        tuned_ld = (k1 * v_robot) - (k2 * omega) 
    else: # Positive omega --> turn left
        tuned_ld = (k1 * v_robot) + (k2 * omega)
    ld = max(tuned_ld, min_ld)  # Ensure lookahead is not below min_ld
    # print(f"Lookahead distance after max: {ld}")
    
    # For testing only
    # return 45

    return ld
