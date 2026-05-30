"""Shared ctypes types and helpers for ArmMotor / ChassisMotor."""
import os
import sys
import time
from ctypes import (
    CDLL,
    CFUNCTYPE,
    POINTER,
    Structure,
    c_float,
    c_int,
    c_int8,
    c_int16,
    c_uint16,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
)
from pathlib import Path
from typing import Any, Callable, List, NamedTuple, Optional, Union

from can.message import Message

TX_FN = CFUNCTYPE(c_int, c_void_p, c_uint32, POINTER(c_uint8), c_uint8)
TxCallback = Callable[[Any, int, Any, int], int]


class _MotorHighSpeedFeedback(Structure):
    _fields_ = [
        ("position", c_float),
        ("velocity", c_float),
        ("current", c_float),
        ("timestamp_ns", c_uint64),
    ]


class _MotorDriverStatusBits(Structure):
    _fields_ = [
        ("undervoltage", c_uint8),
        ("motor_overtemp", c_uint8),
        ("driver_overcurrent", c_uint8),
        ("driver_overtemp", c_uint8),
        ("collision_tripped", c_uint8),
        ("driver_error", c_uint8),
        ("enabled", c_uint8),
        ("stall_tripped", c_uint8),
    ]


class _MotorLowSpeedFeedback(Structure):
    _fields_ = [
        ("bus_voltage_v", c_float),
        ("driver_temp_deg", c_int16),
        ("motor_temp_deg", c_int8),
        ("_pad0", c_uint8),
        ("bus_current", c_float),
        ("status_raw", c_uint8),
        ("status", _MotorDriverStatusBits),
        ("_pad_tail", c_uint8 * 3),
        ("timestamp_ns", c_uint64),
    ]


class _MotorVersionInfo(Structure):
    _fields_ = [
        ("software", c_uint8 * 64),
        ("hardware", c_uint8 * 64),
        ("motor", c_uint8 * 64),
        ("timestamp_ns", c_uint64),
    ]


class DriverStatus(NamedTuple):
    undervoltage: bool
    motor_overtemp: bool
    driver_overcurrent: bool
    driver_overtemp: bool
    collision_tripped: bool
    driver_error: bool
    enabled: bool
    stall_tripped: bool


class HighSpeedFeedback(NamedTuple):
    position: float
    velocity: float
    current: float
    timestamp: float


class LowSpeedFeedback(NamedTuple):
    bus_voltage_v: float
    driver_temp_deg: int
    motor_temp_deg: int
    bus_current: float
    status_raw: int
    status: DriverStatus
    timestamp: float


class VersionInfo(NamedTuple):
    software: str
    hardware: str
    motor: str
    timestamp: float


def ns_to_timestamp(ns: int) -> float:
    return int(ns) * 1e-9


def timestamp_to_ns(timestamp: float) -> int:
    return int(round(timestamp * 1e9))


def c_string(buf: bytes) -> str:
    return buf.split(b"\x00", 1)[0].decode("ascii", "replace")


def driver_status_from_ctypes(raw: _MotorDriverStatusBits) -> DriverStatus:
    return DriverStatus(
        undervoltage=bool(raw.undervoltage),
        motor_overtemp=bool(raw.motor_overtemp),
        driver_overcurrent=bool(raw.driver_overcurrent),
        driver_overtemp=bool(raw.driver_overtemp),
        collision_tripped=bool(raw.collision_tripped),
        driver_error=bool(raw.driver_error),
        enabled=bool(raw.enabled),
        stall_tripped=bool(raw.stall_tripped),
    )


def high_speed_from_ctypes(raw: _MotorHighSpeedFeedback) -> HighSpeedFeedback:
    return HighSpeedFeedback(
        position=float(raw.position),
        velocity=float(raw.velocity),
        current=float(raw.current),
        timestamp=ns_to_timestamp(raw.timestamp_ns),
    )


def low_speed_from_ctypes(raw: _MotorLowSpeedFeedback) -> LowSpeedFeedback:
    return LowSpeedFeedback(
        bus_voltage_v=float(raw.bus_voltage_v),
        driver_temp_deg=int(raw.driver_temp_deg),
        motor_temp_deg=int(raw.motor_temp_deg),
        bus_current=float(raw.bus_current),
        status_raw=int(raw.status_raw),
        status=driver_status_from_ctypes(raw.status),
        timestamp=ns_to_timestamp(raw.timestamp_ns),
    )


def version_from_ctypes(raw: _MotorVersionInfo) -> VersionInfo:
    return VersionInfo(
        software=c_string(bytes(raw.software)),
        hardware=c_string(bytes(raw.hardware)),
        motor=c_string(bytes(raw.motor)),
        timestamp=ns_to_timestamp(raw.timestamp_ns),
    )


def package_lib_dir() -> Path:
    return Path(__file__).resolve().parent


def dynamic_lib_basename(variant: str) -> str:
    if sys.platform == "win32":
        return f"motor_{variant}.dll"
    if sys.platform == "darwin":
        return f"libmotor_{variant}.dylib"
    return f"libmotor_{variant}.so"


def candidate_paths(variant: str, env_key: str) -> List[Path]:
    out: List[Path] = []
    env = os.environ.get(env_key)
    if env:
        out.append(Path(env))
    out.append(package_lib_dir() / dynamic_lib_basename(variant))
    return out


def bind_common(lib: CDLL) -> None:
    h = c_void_p
    lib.motor_create.restype = h
    lib.motor_destroy.argtypes = [h]
    lib.motor_set_tx_callback.argtypes = [h, TX_FN, c_void_p]
    lib.motor_has_tx_callback.argtypes = [h]
    lib.motor_has_tx_callback.restype = c_int
    lib.motor_handle_rx_once.argtypes = [h, c_uint32, POINTER(c_uint8), c_uint8, c_uint64]
    lib.motor_handle_rx_once.restype = c_int
    lib.motor_get_high_speed_feedback.argtypes = [h, c_uint8, POINTER(_MotorHighSpeedFeedback)]
    lib.motor_get_high_speed_feedback.restype = c_int
    lib.motor_get_low_speed_feedback.argtypes = [h, c_uint8, POINTER(_MotorLowSpeedFeedback)]
    lib.motor_get_low_speed_feedback.restype = c_int
    lib.motor_set_target.argtypes = [h, c_uint8, c_float, c_uint8, c_float]
    lib.motor_set_target.restype = c_int
    lib.motor_set_enable.argtypes = [h, c_uint8, c_int, c_int, c_int]
    lib.motor_set_enable.restype = c_int
    lib.motor_set_reset.argtypes = [h, c_uint8, c_int, c_float, POINTER(c_int)]
    lib.motor_set_reset.restype = c_int
    lib.motor_set_clear_error.argtypes = [h, c_uint8, c_uint8, c_float, POINTER(c_int)]
    lib.motor_set_clear_error.restype = c_int
    lib.motor_set_zero_offset.argtypes = [h, c_uint8, c_float, c_int, c_float, POINTER(c_int)]
    lib.motor_set_zero_offset.restype = c_int
    lib.motor_set_profile_acc_dec.argtypes = [h, c_uint8, c_float, c_float]
    lib.motor_set_profile_acc_dec.restype = c_int
    lib.motor_set_profile_vel.argtypes = [h, c_uint8, c_float]
    lib.motor_set_profile_vel.restype = c_int
    lib.motor_set_current_limit.argtypes = [h, c_uint8, c_float]
    lib.motor_set_current_limit.restype = c_int


def bind_arm(lib: CDLL) -> None:
    bind_common(lib)
    h = c_void_p
    lib.motor_set_mit_control.argtypes = [h, c_uint8, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.motor_set_mit_control.restype = c_int
    lib.motor_set_mit_control_crc.argtypes = [h, c_uint8, c_float, c_float, c_float, c_float, c_float, c_float]
    lib.motor_set_mit_control_crc.restype = c_int
    lib.motor_set_collision_threshold.argtypes = [h, c_uint8, c_float, c_float]
    lib.motor_set_collision_threshold.restype = c_int
    lib.motor_get_param.argtypes = [h, c_uint8, c_uint8, c_uint8, c_float, POINTER(c_float)]
    lib.motor_get_param.restype = c_int
    lib.motor_set_param.argtypes = [h, c_uint8, c_uint8, c_uint8, c_float, c_float]
    lib.motor_set_param.restype = c_int
    lib.motor_get_version.argtypes = [h, c_uint8, c_float, POINTER(_MotorVersionInfo)]
    lib.motor_get_version.restype = c_int


def bind_chassis(lib: CDLL) -> None:
    bind_common(lib)
    h = c_void_p
    lib.motor_set_stiffness.argtypes = [h, c_uint8, c_uint16]
    lib.motor_set_stiffness.restype = c_int


class MotorLibCache:
    def __init__(self, variant: str, env_key: str, binder) -> None:
        self._variant = variant
        self._env_key = env_key
        self._binder = binder
        self._lib: Optional[CDLL] = None
        self._error: Optional[BaseException] = None

    def load_error(self) -> Optional[str]:
        return None if self._error is None else str(self._error)

    def reset(self) -> None:
        self._lib = None
        self._error = None

    def load(self, path: Optional[Union[str, Path]] = None) -> Optional[CDLL]:
        candidates = [Path(path)] if path else list(candidate_paths(self._variant, self._env_key))
        last_err: Optional[BaseException] = None
        for p in candidates:
            try:
                if not p.is_file():
                    continue
                lib = CDLL(str(p))
                self._binder(lib)
                self._lib = lib
                self._error = None
                return lib
            except OSError as e:
                last_err = e
        name = dynamic_lib_basename(self._variant)
        self._error = last_err or FileNotFoundError(
            f"{name} not found (set {self._env_key} or build native)"
        )
        self._lib = None
        return None

    def get(self) -> Optional[CDLL]:
        if self._lib is not None:
            return self._lib
        return self.load()


_arm_cache = MotorLibCache("arm", "AGXMOTOR_ARM_LIB", bind_arm)
_chassis_cache = MotorLibCache("chassis", "AGXMOTOR_CHASSIS_LIB", bind_chassis)


def message_timestamp(msg: Message) -> float:
    ts = getattr(msg, "timestamp", None)
    if ts is not None:
        return float(ts)
    return time.time_ns() * 1e-9
