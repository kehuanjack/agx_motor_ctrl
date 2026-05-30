import time
import threading
from agx_motor_ctrl import ArmMotor, CanComm, create_can_comm_config

cfg = create_can_comm_config(channel="can0", interface="socketcan", auto_connect=True)
bus = CanComm(cfg)
motor = ArmMotor(bus)

stop = threading.Event()

def recv_loop() -> None:
    while not stop.is_set():
        try:
            bus.recv()
        except Exception:
            if stop.is_set():
                break

thread = threading.Thread(target=recv_loop, name="can-recv", daemon=True)
thread.start()

time.sleep(0.005)

try:
    node_id = 1
    ver = motor.get_version(node_id, timeout=0.5)
    print(ver)

    print(motor.set_enable(node_id, True))
    print(motor.set_profile_vel(node_id, 3.0))
    print(motor.set_target_position(node_id, 0.0))
finally:
    stop.set()
    motor.close()
    bus.close()