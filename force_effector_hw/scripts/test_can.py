from MMRCS1 import MMRCS1
from datetime import datetime

motor = MMRCS1(1, './copley.eds')
motor.start()
motor.set_mode(4)
s = datetime.now()
for i in range(1000):
    motor.sent_torque(0)
print((datetime.now() - s).seconds)

motor.stop()