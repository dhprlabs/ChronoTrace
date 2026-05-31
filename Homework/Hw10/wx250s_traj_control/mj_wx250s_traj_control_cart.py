import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import utility as ram
from scipy.optimize import fsolve


xml_path = '../trossen_wx250s/scene_mocap.xml' #xml file (assumes this is in the same folder as this file)
#xml_path = '../../mj_models/trossen_wx250s/scene_mocap.xml' #xml file (assumes this is in the same folder as this file)

simend = 100 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

flag_inset_cam = 1;
class InsetCamera:
    def __init__(self, size=300,azimuth=0, elevation=90, distance=1, lookat=None):
        self.size = size
        self.azimuth = azimuth
        self.elevation = elevation
        self.distance = distance
        self.lookat = np.array(lookat if lookat is not None else [0, 0, 0])

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):

    #HINT 4:
    # 0) The joint position  and joint velocity is in data.qpos and data.qvel
    # 1) Compute cartesian position, velocity, acceleration using splines/quintic polynomial
    # 2) Compute quat, quat_dot, quat_doubledot using SLERP
    # 3) Compute omega_world, angular_acceleration_world from quat, quat_dot, quat_doubledot using quat2angvelWorld and quat2angaccWorld in utility
    # 4) Compute q_ref using IK (fsolve) and quat
    # 5) Compute u_ref using Jacobian_inverse times [velocity; omega_world]
    # 6) Compute udot_ref using Jacobian_inverse times [acceleration; angular_acceleration_world] %NOTE this ignores the Jdot term
    # 7) Compute M, C, G
    # 8) For feedforward+feedback, tau is a function of M, C, G, q_ref, u_ref, udot_ref, data.qpos, data,qvel

    data.ctrl = np.zeros(7)

    #put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])
##cam.azimuth = -177.53169145640405 ; cam.elevation = -27.265917875071533 ; cam.distance =  1.2369838995651359
#cam.lookat =np.array([ 0.0 , 0.1 , 0.1 ])
cam.azimuth = 171.04537153187712 ; cam.elevation = -22.225781156321535 ; cam.distance =  1.2369838995651359
cam.lookat =np.array([ 0.0 , 0.1 , 0.1 ])

#a) initialize inset_cameras
inset_cameras = []
#inset_cameras.append(InsetCamera())  # Default values (side view) az=0, el=+/-90, top view
inset_cameras.append(InsetCamera(azimuth=0, elevation=-90, distance=1.25,lookat=[0.25,-0.1, 0]))  # side view (looking at -y) has el=0
#inset_cameras.append(InsetCamera(azimuth=-90, elevation=0, distance=1.25, lookat=[0.3, 0, 0.3]))  # side view (looking at -x) has el=0
#inset_cameras.append(InsetCamera(azimuth=225, elevation=45, distance=1.5, lookat=[0, 0, 0]))  # perspective

inset_cameras[0].size = 400
#inset_cameras[1].size = 300
# inset_cameras[2].size = 400
# inset_cameras[3].size = 300

if (flag_inset_cam==1):
    # Create MjvCamera instances for each inset camera
    inset_mjv_cams = [mj.MjvCamera() for _ in range(len(inset_cameras))]

    # Initialize and configure each inset camera using values from `InsetCamera` instances
    for i, (mjv_cam, inset_cam) in enumerate(zip(inset_mjv_cams, inset_cameras)):
        mj.mjv_defaultCamera(mjv_cam)
        mjv_cam.azimuth = inset_cam.azimuth
        mjv_cam.elevation = inset_cam.elevation
        mjv_cam.distance = inset_cam.distance
        mjv_cam.lookat = inset_cam.lookat


#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

# # 1) End-effector site
site_id = model.site("attachment_site").id
#
# # 2) Name of bodies we wish to apply gravity compensation to.
# #we have not included the body called "base"
body_names = [
"wx250s/shoulder_link",
"wx250s/upper_arm_link",
"wx250s/upper_forearm_link",
"wx250s/lower_forearm_link",
"wx250s/wrist_link",
"wx250s/gripper_link",
"wx250s/left_finger_link",
"wx250s/right_finger_link"
]
body_ids = [model.body(name).id for name in body_names]
# #print(body_ids)
#model.body_gravcomp[body_ids] = 1.0
#
# # 3) Joint / actuators have the same names
joint_names = [
"waist" ,
"shoulder",
"elbow",
"forearm_roll",
"wrist_angle",
"wrist_rotate",
"left_finger",
"right_finger"
]
#
# #4) Joint ids
joint_ids = np.array([model.joint(name).id for name in joint_names])
#

actuator_names = [
"waist" ,
"shoulder",
"elbow",
"forearm_roll",
"wrist_angle",
"wrist_rotate",
"gripper",
]
# # 5) Note that actuator names are the same as joint names in this case.
actuator_ids = np.array([model.actuator(name).id for name in actuator_names])
#
# # 6) Initial joint configuration saved as a keyframe in the XML file.
key_id = model.key("home").id
key_qpos = model.key_qpos[key_id]  #Access qpos
#key_ctrl = model.key_ctrl[key_id] #Access ctrl

#7) mocap id
#get the mocap id of reference
mocap_id = model.body("reference").mocapid[0]
# #mocap_pos = data.mocap_pos[mocap_id]


#guess for IK (feel free to change)
q = np.array([0, 0, 0, 0, 0, 0]);


#reference (start)
x0_ref = 0.4; y0_ref = 0.25; z0_ref = 0.02;
X0_ref = np.array([x0_ref,y0_ref,z0_ref])
euler0 = np.array([0,0,0])
quat0 = ram.euler2quat(euler0)

#HINT1: You need to specifiy final and intermediate (if needed) points for the interpolation
#e.g Xf_ref, quatf etc.


while not glfw.window_should_close(window):
    time_prev = data.time


    while (data.time - time_prev < 1.0/60.0):

        #HINT2: Set the pos/quat of the yellow box here
        #I have put some random values but you need to change these
        pos = X0_ref.copy() + 0.05*np.array([np.sin(data.time),np.sin(2*data.time),0])
        quat = quat0.copy()

        for i in range(0,4):
            data.mocap_quat[mocap_id,i] = quat[i];
            if (i<3):
                data.mocap_pos[mocap_id,i] = pos[i];

        #HINT3: Set the joint position (q) using inverse_kinematics
        #euler = ram.quat2euler(quat)
        #X_ref = np.array([pos[0],pos[1],pos[2],euler[0],euler[1],euler[2]])
        #q,info,ier,msg = fsolve(inverse_kinematics,q,args=(X_ref),full_output=True)

        #Code from HW9
        #data.qpos[:6] = q[:6].copy()
        # data.time += 0.01; #model.opt.timestep
        # mj.mj_forward(model, data)

        #New code for HW10
        controller(model,data)
        mj.mj_step(model, data)


    if (data.time>=simend):
        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')
        print('inset_cameras.append(InsetCamera(azimuth=',cam.azimuth,',elevation=',cam.elevation,\
               ',distance=',cam.distance,',lookat=[',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    #b) show the cameras in the display
    if (flag_inset_cam==1):
        for i, mjv_cam in enumerate(inset_mjv_cams):

            if i == 3:  # Top Left
                inset_x = 10
                inset_y = viewport_height - inset_cameras[i].size - 10
            elif i == 2:  # Bottom Left
                inset_x = 10
                inset_y = 10
            elif i == 1:  # Bottom Right
                inset_x = viewport_width - inset_cameras[i].size - 10
                inset_y = 10
            else:  # Default to Top Right
                inset_x = viewport_width - inset_cameras[i].size - 10
                inset_y = viewport_height - inset_cameras[i].size - 10

            # Define inset viewport
            inset_viewport = mj.MjrRect(inset_x, inset_y, inset_cameras[i].size, inset_cameras[i].size)

            # Update scene for the inset camera
            mj.mjv_updateScene(model, data, opt, None, mjv_cam, mj.mjtCatBit.mjCAT_ALL.value, scene)

            # Render each inset
            mj.mjr_render(inset_viewport, scene, context)


    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
