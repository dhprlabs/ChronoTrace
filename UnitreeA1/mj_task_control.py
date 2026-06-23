import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import task_control as tc 
from inverse_kinematics import inverse_kinematics


xml_path = 'scene.xml'     # xml file (assumes this is in the same folder as this file)
simend = 90                # simulation time
print_camera_config = 0    # set to 1 to print camera config

button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def init_controller(model, data):
    # initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    # put the controller here. This function is called inside the simulation.
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

    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

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
    PRESS_LEFT_SHIFT = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
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

    mj.mjv_moveCamera(model, action, dx/height, dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 * yoffset, scene, cam)

# get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)    # MuJoCo model
data = mj.MjData(model)                       # MuJoCo data
cam = mj.MjvCamera()                          # Abstract camera
opt = mj.MjvOption()                          # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()

primary_monitor = glfw.get_primary_monitor()
window = glfw.create_window(1920, 1080, "Jacobian", primary_monitor, None)
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
cam.azimuth = -51.15
cam.elevation = -22.97
cam.distance = 2.35
cam.lookat = np.array([0.0, 0.0, 0.0])

# initialize the controller
init_controller(model, data)

# set the controller
mj.set_mjcb_control(controller)

initial_angles = model.key("home").qpos
initial_vel = model.key("home").qvel

p_start = np.array([-0.134, 0.486, 0.693])
p_goal  = np.array([0.350, 0.200, 0.600])

q_start = np.array([1.0, 0.0, 0.0, 0.0]) 
q_goal  = np.array([0.7071, 0.7071, 0.0, 0.0]) 

duration = 3.0

data.qpos = initial_angles.copy()
data.qvel = initial_vel.copy()
mj.mj_forward(model, data)     


while not glfw.window_should_close(window):
    t_curr = data.time
    
    p_ref, v_ref, a_ref = tc.position_trajectory(p_start, p_goal, duration, t_curr, profile='quintic')
    quat_ref = tc.orientation_quaternion_slerp(q_start, q_goal, duration, t_curr, profile='quintic')

    print("="*161)
    print("task control trajectory")
    print("="*161)
    print(np.round(p_ref, 3))
    print(np.round(v_ref, 3))
    print(np.round(a_ref, 3))
    print(np.round(quat_ref, 3))

    target_q = inverse_kinematics(p_ref, quat_ref, data.qpos.copy())
    data.ctrl[:] = target_q
    mj.mj_step(model, data)

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # print camera configuration (help to initialize the view)
    if (print_camera_config == 1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation = ',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat = np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()


glfw.terminate()
