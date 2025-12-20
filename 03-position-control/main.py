import math
import os
import time

import mujoco
import mujoco.viewer

curr_dir = os.path.dirname(__file__)
model_path = os.path.join(curr_dir, 'model.xml')
model = mujoco.MjModel.from_xml_path(model_path)

data = mujoco.MjData(model)

# Initialize control variable
counter = 0

def controller(model, data):
    global counter
    counter += 1
    # Instead of sleeping for 0.01s, we control the flow by steps
    # or simply update every time (depends on simulation speed)
    
    # Original logic: control cycle of 180 steps
    cycle = 180
    
    # Create periodic motion
    # Using math.sin is common, but reproducing the original logic (linear increase)
    phase = counter % cycle
    data.ctrl[0] = 0.1 * phase / 180 * math.pi

# Register callback
mujoco.set_mjcb_control(controller)

# macOS note: launch_passive() requires the script to be run with `mjpython`.
# On Linux, you can use launch_passive() with a standard python interpreter.
mujoco.viewer.launch(model, data)
