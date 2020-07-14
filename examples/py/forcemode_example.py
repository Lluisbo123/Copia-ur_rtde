import rtde_control
import time

rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")

task_frame = [0, 0, 0, 0, 0, 0]
selection_vector = [0, 0, 1, 0, 0, 0]
wrench_down = [0, 0, -10, 0, 0, 0]
wrench_up = [0, 0, 10, 0, 0, 0]
force_type = 2
limits = [2, 2, 1.5, 1, 1, 1]

rtde_c.forceModeStart(task_frame, selection_vector, wrench_down, force_type, limits)
print("Going Down!")
time.sleep(2)
print("Going Up!")
rtde_c.forceModeUpdate(wrench_up)
time.sleep(2)
rtde_c.forceModeStop()
rtde_c.stopScript()
