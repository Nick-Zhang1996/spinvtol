# sample controller

# vicon_list: list of 5 previous vicon states (3 element tuple)
#               [(x,y,z,rx,ry,rz),(x,y,z ....]
# Return:
#       thrust_N, flap_rad
def expControl(vicon_list):
    state1 = vicon_list[0]
    return (3000,0.1)

def cvt2pwm(thrust_N,flap_rad,voltage):
    # from fittravel.py
    flapPWM = int((flap_rad+7.542075e-01)/4.994162e-04)
    # from motor.py
    throttlePWM = int((thrust * 11.5 / voltage - -7.65054005442) / 0.00666834489796)
    return (flapPWM,throttlePWM)

