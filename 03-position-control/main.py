import math
import time

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path('model.xml')

data = mujoco.MjData(model)


i = 0
with mujoco.viewer.launch_passive(model, data) as viewer:
    while True:
        i = i + 1
        data.ctrl[0] = 0.1 * i / 180 * math.pi
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
        if i == 180:
            i = 0
