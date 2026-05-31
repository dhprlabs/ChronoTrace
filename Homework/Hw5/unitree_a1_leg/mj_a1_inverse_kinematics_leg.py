import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from inverse_kinematics_analytic import inverse_kinematics_analytic

xml_path = 'unitree_robotics_a1_leg/scene.xml' #xml file (assumes this is in the same folder as this file)
simend = 100 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
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
cam.azimuth = -151.48295541017941 ; cam.elevation = -36.45925186319362 ; cam.distance =  1.402741212145693
cam.lookat =np.array([ 0.0 , 0.0 , 0.1 ])

#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

# Get all ids
# Get various ids

# 1) Get all sites to help in forward kinematics
site_names = [
   "FL_foot",
   "FL_out_shoulder",
   "FL_in_shoulder",
   "FL_elbow",
     ]

site_ids = np.array([model.site(name).id for name in site_names])
#print(site_ids)


# 2) Name of bodies. This is the same as names of actuators
body_names = [
    "FL_hip",
    "FL_thigh",
    "FL_calf",
]
body_ids = [model.body(name).id for name in body_names]
#print(body_ids)
# model.body_gravcomp[body_ids] = 1.0
#
# # 3) Joint names
joint_names = [
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
]
# #4) Get joint_ids
#joint_ids go from 0 to model.nq
joint_ids = np.array([model.joint(name).id for name in joint_names])
#print(joint_ids)

#print(model.jnt_qposadr[1])
#print(model.jnt_dofadr[1])
#print(model.jnt_bodyid[1])
#print(model.jnt_type[1]) #free = 0, ball =1, slide=2,hinge=3
#print(model.joint.id)
# print(model.joints)

#
# 5) Note that actuator names are the same as body names.
# Numbering starts from 1 which skips the "trunk" as there are no actuators there.
actuator_ids = np.array([model.actuator(name).id for name in body_names])
#print(actuator_ids)
#
# 6) Initial joint configuration saved as a keyframe in the XML file.
key_id = model.key("home").id
key_qpos = model.key_qpos[key_id]  #Access qpos
key_ctrl = model.key_ctrl[key_id] #Access ctrl

#7) mocap id
#get the mocap id of reference
mocap_id = model.body("reference").mocapid[0]
# #mocap_pos = data.mocap_pos[mocap_id]


#q = np.array([0, 0, 0.27, 1, 0, 0, 0, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8])
#hip = 0.5; pitch = 0.3; knee = -1.2;
abduction = 0; hip = 0.9; knee = -1.8;
#q = np.array([0, 0, 0.27, 1, 0, 0, 0,-hip, pitch, knee, hip, pitch, knee, -hip, pitch, knee, hip, pitch, knee]);
q = np.array([abduction,hip,knee])
q_ref = q.copy();


while not glfw.window_should_close(window):
    time_prev = data.time
    if (data.time<0.01):
        # Reset the simulation to the initial keyframe.
        mj.mj_resetDataKeyframe(model, data, key_id)

    while (data.time - time_prev < 1.0/60.0):

        #HINT2: Try these three test cases one by one
        x_ref = 0.0;  y_ref = 0.0; z_ref = -0.2;
        #x_ref = 0.1; y_ref = 0; z_ref = -0.1;
        #x_ref = 0; y_ref = 0.1; z_ref = -0.15
        X_ref = np.array([x_ref,y_ref,z_ref])
        q_ref = inverse_kinematics_analytic(X_ref)

        #get position of relevant sites
        out_shoulder_pos = data.site(site_ids[1]).xpos;
        foot_pos = data.site(site_ids[0]).xpos;

        #visual validation (place the red box at the reference)
        data.mocap_pos[mocap_id,0] = out_shoulder_pos[0]+x_ref;
        data.mocap_pos[mocap_id,1] = out_shoulder_pos[1]+y_ref;
        data.mocap_pos[mocap_id,2] = out_shoulder_pos[2]+z_ref;


        #validation:
        print('The below two values should be the same')
        #print the reference
        var = X_ref;
        print(f"(ref) foot position: {np.array2string(var, precision=4, floatmode='fixed', separator=', ')}")
        print('**')

        #print the position of the foot.
        var = foot_pos - out_shoulder_pos #data.site(site_ids[0]).xpos-data.site(site_ids[1]).xpos;
        print(f"(mj)FL_foot: {np.array2string(var, precision=4, floatmode='fixed', separator=', ')}")
        print('*')

        data.qpos = q_ref.copy()
        data.time += model.opt.timestep
        mj.mj_forward(model, data)


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

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
