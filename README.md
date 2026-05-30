# agx_motor_ctrl

Python SDK for Agilex motor drivers over CAN.

- **Protocol** (pack/parse, sync): `libmotor_arm` / `libmotor_chassis` (C++)
- **Transport**: `python-can` + `CanComm` (Linux / Windows / macOS)
- **API**: `ArmMotor` (arm joints), `ChassisMotor` (chassis wheels/steer)

## Install

```shell
pip3 install python-can

git clone https://github.com/kehuanjack/agx_motor_ctrl.git
cd agx_motor_ctrl
pip3 install .
```

`pip3 install` runs `motor_install_hook.py`, which downloads **both** prebuilt libraries for your platform from GitHub Releases (`libmotor-arm-*` and `libmotor-chassis-*`), unless `AGXMOTOR_SKIP_DOWNLOAD=1` and the files are already under `agx_motor_ctrl/motor/`.

Windows with Agilex CANDO: install [python-can-agx-cando](https://github.com/agilexrobotics/python-can-agx-cando) and use `interface="agx_cando"`.

Installing this package may download precompiled **libmotor** binaries; see [License](#license) below.

## CAN setup (Linux)

```shell
sudo ip link set can0 up type can bitrate 1000000
```

## Quick start (Arm)

`ArmMotor` uses the arm joint protocol (position [rad], velocity [rad/s], MIT, parameters, version, etc.). One controller instance handles one protocol; up to **15** motors per bus (`node_id` 1–15).

`ArmMotor(bus)` registers libmotor **tx** on `CanComm`. **RX is passive:** you must run a `recv()` loop; frames reach `handle_rx_once` via your dispatch logic (see [Mixed protocols](#mixed-protocols-on-one-can-bus) if both arm and chassis share a bus).

```python
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

    hs = motor.get_high_speed_feedback(node_id)
    if hs is not None:
        print(hs.position, hs.velocity, hs.current)
finally:
    stop.set()
    motor.close()
    bus.close()
```

## Quick start (Chassis)

`ChassisMotor` uses the chassis protocol. High-speed feedback: `position` [INC], `velocity` [RPM]. It adds `set_stiffness` and does **not** expose MIT, parameters, version, or collision APIs.

```python
import time
import threading

from agx_motor_ctrl import ChassisMotor, CanComm, create_can_comm_config

cfg = create_can_comm_config(channel="can0", interface="socketcan", auto_connect=True)
bus = CanComm(cfg)
motor = ChassisMotor(bus)

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
    node_id = 2
    motor.set_enable(node_id, True)

    hs = motor.get_high_speed_feedback(node_id)
    if hs is not None:
        # position [INC], velocity [RPM], current [A]
        print(hs.position, hs.velocity, hs.current)
finally:
    stop.set()
    motor.close()
    bus.close()
```

Low-speed `status` on chassis maps sensor/homed to `collision_tripped` / `stall_tripped` field names in the shared struct layout.

## Mixed protocols on one CAN bus

Arm and chassis motors may share one CAN interface if **`node_id` values do not overlap**. Do **not** call `attach_can_comm` on both `ArmMotor` and `ChassisMotor` for the same `CanComm` (only one RX callback is allowed).

Register **one** `bus.set_callback` and dispatch by node ID:

```python
ARM_IDS = {1, 2, 3}
CHASSIS_IDS = {4, 5}

def on_frame(msg):
    nid = msg.arbitration_id & 0x0F  # adjust to your ID layout
    if nid in ARM_IDS:
        arm.handle_rx_once(msg=msg)
    elif nid in CHASSIS_IDS:
        chassis.handle_rx_once(msg=msg)

bus.set_callback(on_frame)
# arm / chassis: register tx only via set_tx_callback, not attach_can_comm
```

## Examples

Examples are split by protocol under [`examples/arm/`](examples/arm/) and [`examples/chassis/`](examples/chassis/).

**Arm** (`ArmMotor`):

| Script | Purpose |
| --- | --- |
| [`enable.py`](examples/arm/enable.py) | Read version, enable |
| [`disable.py`](examples/arm/disable.py) | Read version, disable |
| [`go_zero.py`](examples/arm/go_zero.py) | Enable, profile velocity, move to position 0 [rad] |
| [`get_high_speed.py`](examples/arm/get_high_speed.py) | Poll high-speed feedback [rad, rad/s, A] |
| [`get_low_speed.py`](examples/arm/get_low_speed.py) | Poll low-speed feedback |

**Chassis** (`ChassisMotor`):

| Script | Purpose |
| --- | --- |
| [`enable.py`](examples/chassis/enable.py) | Enable |
| [`disable.py`](examples/chassis/disable.py) | Disable |
| [`set_rpm.py`](examples/chassis/set_rpm.py) | Enable, stiffness, profile accel, target RPM |
| [`get_high_speed.py`](examples/chassis/get_high_speed.py) | Poll high-speed feedback [INC, RPM, A] |
| [`get_low_speed.py`](examples/chassis/get_low_speed.py) | Poll low-speed feedback |

Run after CAN is up, e.g. `python3 examples/arm/enable.py` or `python3 examples/chassis/enable.py`.

## Environment

| Variable | Purpose |
| --- | --- |
| `AGXMOTOR_ARM_LIB` | Runtime path to `libmotor_arm` (`.so` / `.dll` / `.dylib`) |
| `AGXMOTOR_CHASSIS_LIB` | Runtime path to `libmotor_chassis` |
| `AGXMOTOR_SKIP_DOWNLOAD` | Skip Release download at `pip3 install` |
| `AGXMOTOR_GITHUB_REPO` | Release repo (default `kehuanjack/agx_motor_ctrl`) |

After a local native build in the private repo, libraries are copied to `agx_motor_ctrl/motor/` as `libmotor_arm.so` and `libmotor_chassis.so`.

## License

Copyright (C) 2026 Agilex Robotics Co., Ltd.

**Python SDK** — [MIT](LICENSE).

**Precompiled libmotor** (`.so` / `.dll` / `.dylib`, including Release downloads and `pip3 install`):

- Proprietary; for use with Agilex motor drivers and this SDK only
- No source code (including CAN protocol implementation)
- No reverse engineering or redistribution without permission
- Provided as-is

Your application code is not required to be open source.
