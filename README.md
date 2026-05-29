# agx_motor_ctrl

Python SDK for Agilex motor drivers over CAN.

- **Protocol** (pack/parse, sync): `libmotor` (C++)
- **Transport**: `python-can` + `CanComm` (Linux / Windows / macOS)
- **API**: `Motor` — send commands, parse feedback by `node_id`

## Install

```shell
pip3 install python-can

git clone https://github.com/kehuanjack/agx_motor_ctrl.git
cd agx_motor_ctrl
pip3 install .
```

Windows with Agilex CANDO: install [python-can-agx-cando](https://github.com/agilexrobotics/python-can-agx-cando) and use `interface="agx_cando"`.

Installing this package may download precompiled **libmotor** binaries; see [License](#license) below.

## CAN setup (Linux)

```shell
sudo ip link set can0 up type can bitrate 1000000
```

## Quick start

One `Motor` per CAN bus (`node_id` 1–15). `Motor(bus)` registers libmotor **tx** and **rx** callbacks on `CanComm`.

**RX is passive:** `Motor` does not call `recv()` for you. Your application must run a loop that calls `bus.recv()`; received frames are forwarded to `handle_rx_once` via the `CanComm` callback. Blocking APIs such as `get_version` / `get_param` wait until matching frames arrive on that path.

```python
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

    hs = motor.get_high_speed_feedback(node_id)
    if hs is not None:
        print(hs.position, hs.velocity, hs.current)
finally:
    stop.set()
    motor.close()
    bus.close()
```

See [`examples/`](examples/) for runnable scripts:

| Script | Purpose |
| --- | --- |
| [`enable.py`](examples/enable.py) | Read version, enable motor |
| [`disable.py`](examples/disable.py) | Read version, disable motor |
| [`go_zero.py`](examples/go_zero.py) | Enable, set profile velocity, move to position 0 |
| [`get_high_speed.py`](examples/get_high_speed.py) | Poll high-speed feedback (`position`, `velocity`, `current`) |
| [`get_low_speed.py`](examples/get_low_speed.py) | Poll low-speed feedback (voltage, temperature, status) |

Run from the package root after CAN is up, e.g. `python3 examples/enable.py`.

## Environment

| Variable | Purpose |
| --- | --- |
| `AGXMOTOR_LIB` | Runtime path to libmotor |
| `AGXMOTOR_SKIP_DOWNLOAD` | Skip Release download at pip3 install |
| `AGXMOTOR_GITHUB_REPO` | Release repo (default `kehuanjack/agx_motor_ctrl`) |

## License

Copyright (C) 2026 Agilex Robotics Co., Ltd.

**Python SDK** — [MIT](LICENSE).

**Precompiled libmotor** (`.so` / `.dll` / `.dylib`, including Release downloads and `pip3 install`):

- Proprietary; for use with Agilex motor drivers and this SDK only
- No source code (including CAN protocol implementation)
- No reverse engineering or redistribution without permission
- Provided as-is

Your application code is not required to be open source.

