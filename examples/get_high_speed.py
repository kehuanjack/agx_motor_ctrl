import time
import threading
from agx_motor_ctrl import CanComm, Motor, create_can_comm_config

cfg = create_can_comm_config(channel="can0", interface="socketcan", auto_connect=True)
bus = CanComm(cfg)
motor = Motor(bus)

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

    while True:
        hs = motor.get_high_speed_feedback(node_id)
        if hs is not None:
            print(hs.position, hs.velocity, hs.current, hs.timestamp)

        time.sleep(0.005)
        
finally:
    stop.set()
    motor.close()
    bus.close()