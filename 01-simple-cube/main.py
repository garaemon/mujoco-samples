import time

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path('model.xml')

data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while True:
        mujoco.mj_step(model, data)
        viewer.sync()

viewer.close()
