import dmp
import numpy as np
import math

packet = np.random.bytes(42)
packet = bytes.fromhex(
    "3fffd2630003eca3fff84568ffb415b9fffffb28000007bdfffff665000ab2330004947c202e2769b26a"
)
q = dmp.quaternion(packet)
print(q)
ypr = dmp.yawPitchRoll(packet)
print("YAW: %3.1f" % (ypr[0] * 180 / math.pi))
print("PITCH: %3.1f" % (ypr[1] * 180 / math.pi))
print("ROLL: %3.1f" % (ypr[2] * 180 / math.pi))
