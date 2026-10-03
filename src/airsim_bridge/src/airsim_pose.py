import airsim
import os
import time

# connect to the AirSim simulator
client = airsim.MultirotorClient("192.168.138.1")
client.confirmConnection()

while True:
    state = client.simGetVehiclePose()
    # state = client.getMultirotorState()
    position = state.position
    print(
        f"✅ 无人机位置: X={position.x_val:.2f}, Y={position.y_val:.2f}, Z={position.z_val:.2f}"
    )
    # print(state)
    time.sleep(0.02)