import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import sys
import matplotlib.pyplot as plt  # Added for plotting

# Import your custom modules
from quintic_interpolation import quintic_interpolation
from tracking_controllers import pd_feedforward_control, inverse_dynamics_control

# --- SIMULATION PARAMETERS ---
t0 = 0
tf = 4
xml_path = 'scene_torque.xml'
simend = 5
print_camera_config = 0

# --- CONTROL GAINS ---
KP_GAINS = np.diag([500.0, 500.0, 500.0, 200.0, 200.0, 200.0])
KD_GAINS = np.diag([80.0, 80.0, 80.0, 30.0, 30.0, 30.0])

# Global container to bridge the main loop and the MuJoCo control callback
class TargetState:
    q = np.zeros(6)
    qd = np.zeros(6)
    qdd = np.zeros(6)
    active_controller = 'idc' # Change to 'pd_ff' to test Feedforward

# --- DATA LOGGING ARRAYS ---
# These lists will store the trajectory history for plotting later
time_history = []
q_desired_history = []
q_actual_history = []

# --- CALLBACKS ---
def init_controller(model, data):
    data.qpos[:6] = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]
    mj.mj_forward(model, data)

def controller(model, data):
    if TargetState.active_controller == 'idc':
        tau = inverse_dynamics_control(
            model, data, 
            TargetState.q, TargetState.qd, TargetState.qdd, 
            KP_GAINS, KD_GAINS
        )
    else:
        tau = pd_feedforward_control(
            model, data, 
            TargetState.q, TargetState.qd, TargetState.qdd, 
            KP_GAINS, KD_GAINS
        )
    data.ctrl[:6] = tau

# --- MOUSE/KEYBOARD CALLBACKS ---
button_left = False
button_middle = False
button_right = False
lastx, lasty = 0, 0

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        init_controller(model, data)

def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right
    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    global lastx, lasty, button_left, button_middle, button_right
    dx, dy = xpos - lastx, ypos - lasty
    lastx, lasty = xpos, ypos

    if not (button_left or button_middle or button_right):
        return

    height = glfw.get_window_size(window)[1]
    mod_shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or 
                 glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)

    if button_right:
        action = mj.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        action = mj.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height, dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 * yoffset, scene, cam)


# --- INITIALIZATION ---
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)
cam = mj.MjvCamera()
opt = mj.MjvOption()
opt.frame = mj.mjtFrame.mjFRAME_SITE

glfw.init()
window = glfw.create_window(1200, 900, "Trajectory Tracking Performance", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

cam.azimuth = -51.15
cam.elevation = -22.97
cam.distance = 2.35
cam.lookat = np.array([0.0, 0.0, 0.0])

init_controller(model, data)
mj.set_mjcb_control(controller)

# Trajectory Goals
q0 = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0])
qf = np.array([np.pi, -np.pi, 0, 0, 0, np.pi])

# --- MAIN LOOP ---
while not glfw.window_should_close(window) and data.time <= simend:
    
    t_eval = np.clip(data.time, t0, tf)
    q_des, qd_des, qdd_des = quintic_interpolation(t_eval, t0, tf, q0, qf)
    
    TargetState.q = q_des.copy()
    TargetState.qd = qd_des.copy()
    TargetState.qdd = qdd_des.copy()

    # --- RECORD DATA DATA POINTS ---
    time_history.append(data.time)
    q_desired_history.append(q_des.copy())
    q_actual_history.append(data.qpos[:6].copy())

    mj.mj_step(model, data)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()

# --- GENERATE PERFORMANCE GRAPHS ---
# Convert python lists to numpy arrays for matrix slice operations
t_arr = np.array(time_history)
q_des_arr = np.array(q_desired_history)
q_act_arr = np.array(q_actual_history)

# Setup a 2x3 subplot layout grid for the 6 joints
fig, axs = plt.subplots(2, 3, figsize=(14, 8))
fig.suptitle(f"UR5e Trajectory Profile Analysis (Controller Mode: {TargetState.active_controller.upper()})", fontsize=16, fontweight='bold')

for i in range(6):
    row = i // 3
    col = i % 3
    
    # Plot desired mathematical trajectory path (Dashed Red Line)
    axs[row, col].plot(t_arr, np.degrees(q_des_arr[:, i]), 'r--', linewidth=2, label='Desired Target')
    # Plot actual tracked simulation path (Solid Blue Line)
    axs[row, col].plot(t_arr, np.degrees(q_act_arr[:, i]), 'b-', linewidth=1.5, label='Actual Encoder', alpha=0.8)
    
    axs[row, col].set_title(f'Joint {i+1} Performance')
    axs[row, col].set_xlabel('Time (seconds)')
    axs[row, col].set_ylabel('Position (Degrees)')
    axs[row, col].grid(True, linestyle=':', alpha=0.6)
    axs[row, col].legend(loc='best')

plt.tight_layout()
plt.show()