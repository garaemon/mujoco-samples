import math
import mujoco
import mujoco_viewer

model = mujoco.MjModel.from_xml_path('model.xml')

data = mujoco.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)

i = 0
while True:
    i = i + 1
    data.ctrl[0] = 0.1 * i / 180 * math.pi
    mujoco.mj_step(model, data)

    viewer.render()

    if i == 180:
        i = 0

viewer.close()
