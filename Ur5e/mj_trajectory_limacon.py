import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import utility
from forward_kinematics import forward_kinematics
from inverse_kinematics import inverse_kinematics
from plot_and_save_trajectory import save_trajectory_plot, log_trajectory
from scipy.optimize import fsolve


xml_path = 'scene.xml'     # xml file (assumes this is in the same folder as this file)
simend = 10                # simulation time
print_camera_config = 0    # set to 1 to print camera config

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0


def limacon(a, b, t):
    x = (a + b * np.cos(2*np.pi*t)) * np.cos(2*np.pi*t)
    y = (a + b * np.cos(2*np.pi*t)) * np.sin(2*np.pi*t)
    
    return x, y


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
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    if (not button_left) and (not button_middle) and (not button_right):
        return

    width, height = glfw.get_window_size(window)

    PRESS_LEFT_SHIFT = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

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

# show attachment site frame
opt.frame = mj.mjtFrame.mjFRAME_SITE

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
cam.azimuth = -51.15
cam.elevation = -22.97
cam.distance = 2.35
cam.lookat = np.array([0.0, 0.0, 0.0])

# initialize the controller
init_controller(model, data)

# set the controller
mj.set_mjcb_control(controller)

q = model.key("home").qpos

target_ee = [-0.134, 0.492, 0.488, 3.14, 0, 0]
x_init = target_ee[0]
y_init = target_ee[1]
z_fixed = target_ee[2]

while not glfw.window_should_close(window):
    t = data.time

    x, y = limacon(0.1, 0.1, t)
    target_ee[0] = x + x_init
    target_ee[1] = y + y_init
    target_ee[2] = z_fixed
    
    new_joint_angles = fsolve(inverse_kinematics, q, args=target_ee)
    data.qpos = new_joint_angles
    
    mj.mj_forward(model, data)     

    mj_ee_pose = data.site("attachment_site").xpos.copy()
    log_trajectory(mj_ee_pose)
    print("Target:", target_ee[:3])
    print("Actual:", mj_ee_pose)
    print("="*85)

    q = new_joint_angles.copy()

    mj.mj_step(model, data)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    if (print_camera_config == 1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation = ',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat = np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

save_trajectory_plot()
glfw.terminate()