import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from planar_jacobian import planar_jacobian
from planar_jacobian_torque import planar_jacobian_torque

xml_path = 'planar.xml'        # xml file (assumes this is in the same folder as this file)
simend = 10                    # simulation time
print_camera_config = 0        # set to 1 to print camera config

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0


def init_controller(model,data):
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
cam.azimuth = 90
cam.elevation = -90
cam.distance = 6.50
cam.lookat = np.array([0.0, 0.0, 0.0])

# initialize the controller
init_controller(model, data)

# set the controller
mj.set_mjcb_control(controller)

planar_qpos = model.key("home").qpos
planar_qvel = model.key("home").qvel

while not glfw.window_should_close(window):
    t = data.time

    data.qpos = planar_qpos.copy()          
    data.qvel = planar_qvel.copy()
    
    q = data.qpos.copy()
    u = data.qvel.copy()
    
    # method-1 => jacobian
    # jacp = np.zeros(shape=(3,3))
    # jacr = np.zeros(shape=(3,3))
    # site_id = model.site("end_effector").id
    # mj.mj_jacSite(model, data, jacp, jacr, site_id)
    # mj_j_sol = np.vstack([jacp, jacr])
    
    # method-2 => jacobian
    # jacp = np.zeros(shape=(3,3))
    # jacr = np.zeros(shape=(3,3))
    # body_id = model.body("link_3").id
    # point_pos = data.site("end_effector").xpos
    # mj.mj_jac(model, data, jacp, jacr, point_pos, body_id)
    # mj_j_sol = np.vstack([jacp, jacr])
    
    # py jacobian
    # j_sol = planar_jacobian(q_dot=q)
    
    # print("="*70)
    # print("mj jacobian")
    # print(mj_j_sol)
    # print("py jacobian")
    # print(j_sol.j_e)

    # jacobian torque
    jacp = np.zeros(shape=(3,3))
    jacr = np.zeros(shape=(3,3))
    body_id = model.body("link_2").id
    mj.mj_jacBodyCom(model, data, jacp, jacr, body_id)
    mj_jg2 = np.vstack([jacp, jacr])

    # py jacobian torque of point g2
    j_sol = planar_jacobian_torque(q_dot=q)
    
    print("="*70)
    print("mj jacobian torque of point g2")
    print(mj_jg2)
    print("py jacobian torque of point g2")
    print(j_sol.jg2)
    

    mj.mj_forward(model, data)   
    mj.mj_step(model, data)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()


glfw.terminate()