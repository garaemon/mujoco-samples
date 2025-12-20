import os
import time

import mujoco
import mujoco.viewer

curr_dir = os.path.dirname(__file__)
model_path = os.path.join(curr_dir, 'model.xml')
model = mujoco.MjModel.from_xml_path(model_path)

data = mujoco.MjData(model)

# macOS note: launch_passive() requires the script to be run with `mjpython`.
# On Linux, you can use launch_passive() with a standard python interpreter.
mujoco.viewer.launch(model, data)
