import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import globals
from state_machine import state_machine
from joint_control import quintic_trajectory
from leg_inverse_kinematics import leg_inverse_kinematics
from leg_forward_kinematics import leg_forward_kinematics
from torque_control import compute_leg_torque

# --- TARGET VELOCITY CONFIGURATION ---
# Set these to control the direction of the robot!
V_X = 0.25   # Set positive to move forward, negative to move backward (m/s)
V_Y = 0.0    # Set positive to move left, negative to move right (m/s)
# -------------------------------------

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
window = glfw.create_window(1920, 1080, "UnitreeA1", primary_monitor, None)
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
cam.azimuth = 77.60000000000011
cam.elevation = -28.638131510416667
cam.distance = 1.612334067683594
cam.lookat = np.array([-0.008101523900779548, 0.001781235495578865, 0.14461303456841276])

# initialize the controller
init_controller(model, data)

# set the controller
mj.set_mjcb_control(controller)

globals.init()
data.qpos = model.key("home").qpos.copy()
ik_seed_angles = data.qpos[7:].copy()
mj.mj_forward(model, data)

control_mode = "joint" # (joint / torque)


while not glfw.window_should_close(window):
    # globals.time = data.time
    # state_machine()
    # target_qpos_legs = np.zeros(12)
    
    # for leg_no in range(4):
    #     t_start = globals.t_fsm[leg_no]
    #     t_step = globals.params.t_step
    #     t_half = 0.5 * t_step
    #     lz_0 = globals.params.lz_0
    #     h_c = globals.params.hc_l
        
    #     if (globals.fsm[leg_no] == globals.params.fsm_stand):
    #         phi = 0.0
    #     else:
    #         phi = np.clip((globals.time - t_start) / t_step, 0.0, 1.0)
            
    #     if globals.fsm[leg_no] == globals.params.fsm_swing:
    #         if phi <= 0.5:
    #             t_local = globals.time - t_start
    #             q_z, _, _ = quintic_trajectory(q0=lz_0, qf=lz_0 + h_c, tf=t_half, t=t_local)
    #             globals.lz_ref[leg_no] = q_z
    #         else:
    #             t_local = globals.time - (t_start + t_half)
    #             q_z, _, _ = quintic_trajectory(q0=lz_0 + h_c, qf=lz_0, tf=t_half, t=t_local)
    #             globals.lz_ref[leg_no] = q_z
    #     else:
    #         globals.lz_ref[leg_no] = lz_0

    #     x_target = 0.0
    #     y_target = 0.08505 if leg_no in [0, 3] else -0.08505
    #     z_target = globals.lz_ref[leg_no]
        
    #     target_vector = np.array([x_target, y_target, z_target])
    #     leg_seed = ik_seed_angles[leg_no*3 : (leg_no+1)*3]
        
    #     leg_angles = leg_inverse_kinematics(target_vector, leg_no, leg_seed)
    #     target_qpos_legs[leg_no*3 : (leg_no+1)*3] = leg_angles

    # data.qpos[7:] = target_qpos_legs
    
    # mj.mj_forward(model, data)

    globals.time = data.time
    state_machine()
    
    # 1. ANCHOR SANDBOX LAYER
    # Freeze Height, Roll, Pitch, and Yaw so the robot stays upright while sliding in X and Y
    data.qpos[2] = 0.245  # Keep the torso at a stable standing height
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0]) # Lock orientation flat
    data.qvel[2:6] = 0.0 # Nullify vertical and rotational velocity drifts
    
    # Stride configurations calculated dynamically from target speeds
    Sx = V_X * globals.params.t_step
    Sy = V_Y * globals.params.t_step
    
    applied_torques = np.zeros(12)
    
    # 2. Loop Through All 4 Legs to Calculate reference profiles and Torques
    for leg_no in range(4):
        t_start = globals.t_fsm[leg_no]
        t_step = globals.params.t_step
        t_half = 0.5 * t_step
        lz_0 = globals.params.lz_0
        h_c = globals.params.hc_l
        
        # Define baseline hip center attachment offset coordinates
        y_hip_default = 0.08505 if leg_no in [0, 3] else -0.08505
        
        if globals.fsm[leg_no] == globals.params.fsm_stand:
            phi = 0.0
        else:
            phi = np.clip((globals.time - t_start) / t_step, 0.0, 1.0)
            
        # --- TRAJECTORY GENERATION BLOCK ---
        if globals.fsm[leg_no] == globals.params.fsm_swing:
            # Swing Phase: Lift the leg up/down and translate horizontally forward
            if phi <= 0.5:
                t_local = globals.time - t_start
                s, _, _ = quintic_trajectory(q0=0.0, qf=1.0, tf=t_half, t=t_local)
                z_target = lz_0 + (s * h_c)
            else:
                t_local = globals.time - (t_start + t_half)
                s, _, _ = quintic_trajectory(q0=1.0, qf=0.0, tf=t_half, t=t_local)
                z_target = lz_0 + (s * h_c)
                
            # Move footprint from back (-S/2) to front (+S/2) relative to the hip
            s_full, _, _ = quintic_trajectory(q0=0.0, qf=1.0, tf=t_step, t=globals.time - t_start)
            x_target = -Sx/2 + (s_full * Sx)
            y_target = y_hip_default - Sy/2 + (s_full * Sy)
            
        else:
            # Stance Phase: Hold foot flat on the ground and sweep backward to propel body
            z_target = lz_0
            x_target = Sx/2 - (phi * Sx)
            y_target = y_hip_default + Sy/2 - (phi * Sy)

        # 3. COMPUTE TARGET JOINT ANGLES VIA INVERSE KINEMATICS
        target_vector = np.array([x_target, y_target, z_target])
        leg_seed = ik_seed_angles[leg_no*3 : (leg_no+1)*3]
        leg_angles_d = leg_inverse_kinematics(target_vector, leg_no, leg_seed)
        
        # 4. EXTRACT LIVE PHYSICAL FEEDBACK VARIABLES FROM MUJOCO
        q_leg_curr = data.qpos[7 + leg_no*3 : 7 + (leg_no+1)*3]
        u_leg_curr = data.qvel[6 + leg_no*3 : 6 + (leg_no+1)*3]
        p_leg_curr = leg_forward_kinematics(q_leg_curr, leg_no)
        
        # 5. EVALUATE TORQUE LAW
        leg_torque = compute_leg_torque(
            q_leg=q_leg_curr,
            u_leg=u_leg_curr,
            q_d=leg_angles_d,
            u_d=np.zeros(3),
            leg_no=leg_no,
            fsm_state=globals.fsm[leg_no],
            fsm_swing=globals.params.fsm_swing,
            p_curr=p_leg_curr,
            p_ref=target_vector,
            mass=globals.mass,
            gravity=globals.gravity
        )
        applied_torques[leg_no*3 : (leg_no+1)*3] = leg_torque

    # 6. INJECT TORQUE FORCES TO MOTOR ACTUATORS AND ADVANCE PHYSICS
    data.ctrl[:] = applied_torques
    mj.mj_step(model, data)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    if (print_camera_config == 1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation = ',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat = np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    cam.lookat[0] = data.qpos[0]
    cam.lookat[1] = data.qpos[1]

    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

    # data.time += 0.0025


glfw.terminate()