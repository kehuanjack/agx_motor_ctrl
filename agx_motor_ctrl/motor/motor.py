"""电机控制 Python 封装（ctypes + libmotor）。"""
import os
import sys
import time
from enum import Enum
from ctypes import (
    CDLL,
    CFUNCTYPE,
    POINTER,
    Structure,
    byref,
    cast,
    c_float,
    c_int,
    c_int8,
    c_int16,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
)
from pathlib import Path
from typing import Any, Callable, List, NamedTuple, Optional, Union

from can.message import Message

from agx_motor_ctrl.comm.can_comm import CanComm

TX_FN = CFUNCTYPE(c_int, c_void_p, c_uint32, POINTER(c_uint8), c_uint8)  # libmotor 发送回调类型
TxCallback = Callable[[Any, int, Any, int], int]  # TX_FN 实例的类型注解

_motor_lib: Optional[CDLL] = None
_motor_lib_error: Optional[BaseException] = None


class _MotorHighSpeedFeedback(Structure):
    """ctypes：高速反馈结构体。"""

    _fields_ = [
        ("position", c_float),
        ("velocity", c_float),
        ("current", c_float),
        ("timestamp_ns", c_uint64),
    ]


class _MotorDriverStatusBits(Structure):
    """ctypes：驱动器状态位结构体。"""

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
    """ctypes：低速反馈结构体。"""

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
    """ctypes：版本信息结构体。"""

    _fields_ = [
        ("software", c_uint8 * 64),
        ("hardware", c_uint8 * 64),
        ("motor", c_uint8 * 64),
        ("timestamp_ns", c_uint64),
    ]


class DriverStatus(NamedTuple):
    """驱动器状态位（由低速反馈 ``status_raw`` 按位解码）。

    成员
    ----
    undervoltage : bool
        欠压告警。
    motor_overtemp : bool
        电机过温告警。
    driver_overcurrent : bool
        驱动器过流告警。
    driver_overtemp : bool
        驱动器过温告警。
    collision_tripped : bool
        碰撞保护已触发。
    driver_error : bool
        驱动器错误（具体含义以固件为准）。
    enabled : bool
        驱动器使能状态指示。
    stall_tripped : bool
        堵转保护已触发。
    """

    undervoltage: bool
    motor_overtemp: bool
    driver_overcurrent: bool
    driver_overtemp: bool
    collision_tripped: bool
    driver_error: bool
    enabled: bool
    stall_tripped: bool


class HighSpeedFeedback(NamedTuple):
    """高速反馈快照（由 :meth:`Motor.get_high_speed_feedback` 返回）。

    成员
    ----
    position : float
        关节位置 [rad]。
    velocity : float
        关节角速度 [rad/s]。
    current : float
        相电流 [A]。
    timestamp : float
        本条反馈的时间戳 [s]，ns precision。
    """

    position: float
    velocity: float
    current: float
    timestamp: float


class LowSpeedFeedback(NamedTuple):
    """低速补充反馈快照（由 :meth:`Motor.get_low_speed_feedback` 返回）。

    成员
    ----
    bus_voltage_v : float
        母线电压 [V]。
    driver_temp_deg : int
        驱动器温度 [°C]。
    motor_temp_deg : int
        电机温度 [°C]。
    bus_current : float
        母线电流 [A]。
    status_raw : int
        原始状态字节（0–255），与 ``status`` 各布尔位一一对应。
    status : DriverStatus
        由 ``status_raw`` 解码后的状态位集合。
    timestamp : float
        本条反馈的时间戳 [s]，ns precision。
    """

    bus_voltage_v: float
    driver_temp_deg: int
    motor_temp_deg: int
    bus_current: float
    status_raw: int
    status: DriverStatus
    timestamp: float


class VersionInfo(NamedTuple):
    """版本信息（由 :meth:`Motor.get_version` 阻塞查询返回）。

    成员
    ----
    software : str
        软件版本字符串，例如 ``"v1.2.3"``。
    hardware : str
        硬件版本字符串，例如 ``"v1.0.0"``。
    motor : str
        电机型号/版本字符串，例如 ``"v2.1.0"``。
    timestamp : float
        版本应答的时间戳 [s]，ns precision。
    """

    software: str
    hardware: str
    motor: str
    timestamp: float


def _ns_to_timestamp(ns: int) -> float:
    # ns -> s [s], ns precision
    return int(ns) * 1e-9


def _timestamp_to_ns(timestamp: float) -> int:
    # s -> ns
    return int(round(timestamp * 1e9))


def _c_string(buf: bytes) -> str:
    # 从 C 字符串缓冲区解码为 Python str
    return buf.split(b"\x00", 1)[0].decode("ascii", "replace")


def _driver_status_from_ctypes(raw: _MotorDriverStatusBits) -> DriverStatus:
    # ctypes 状态位转 DriverStatus
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


def _high_speed_from_ctypes(raw: _MotorHighSpeedFeedback) -> HighSpeedFeedback:
    # ctypes 高速反馈转 HighSpeedFeedback
    return HighSpeedFeedback(
        position=float(raw.position),
        velocity=float(raw.velocity),
        current=float(raw.current),
        timestamp=_ns_to_timestamp(raw.timestamp_ns),
    )


def _low_speed_from_ctypes(raw: _MotorLowSpeedFeedback) -> LowSpeedFeedback:
    # ctypes 低速反馈转 LowSpeedFeedback
    return LowSpeedFeedback(
        bus_voltage_v=float(raw.bus_voltage_v),
        driver_temp_deg=int(raw.driver_temp_deg),
        motor_temp_deg=int(raw.motor_temp_deg),
        bus_current=float(raw.bus_current),
        status_raw=int(raw.status_raw),
        status=_driver_status_from_ctypes(raw.status),
        timestamp=_ns_to_timestamp(raw.timestamp_ns),
    )


def _version_from_ctypes(raw: _MotorVersionInfo) -> VersionInfo:
    # ctypes 版本信息转 VersionInfo
    return VersionInfo(
        software=_c_string(bytes(raw.software)),
        hardware=_c_string(bytes(raw.hardware)),
        motor=_c_string(bytes(raw.motor)),
        timestamp=_ns_to_timestamp(raw.timestamp_ns),
    )


def _motor_dynamic_lib_name() -> str:
    # 当前平台的 libmotor 动态库文件名
    if sys.platform == "win32":
        return "motor.dll"
    if sys.platform == "darwin":
        return "libmotor.dylib"
    return "libmotor.so"


def _candidate_paths() -> List[Path]:
    # libmotor 候选搜索路径
    out: List[Path] = []
    env = os.environ.get("AGXMOTOR_LIB")
    if env:
        out.append(Path(env))
    out.append(Path(__file__).resolve().parent / _motor_dynamic_lib_name())
    return out


def _bind(lib: CDLL) -> None:
    # 绑定 libmotor 的 ctypes 函数签名
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


def _motor_library_load_error() -> Optional[str]:
    # 返回最近一次 libmotor 加载失败的错误信息
    if _motor_lib_error is None:
        return None
    return str(_motor_lib_error)


def _load_motor_library(path: Optional[Union[str, Path]] = None) -> Optional[CDLL]:
    # 加载 libmotor 并绑定 ctypes 符号
    global _motor_lib, _motor_lib_error
    candidates = [Path(path)] if path else list(_candidate_paths())
    last_err: Optional[BaseException] = None
    for p in candidates:
        try:
            if not p.is_file():
                continue
            lib = CDLL(str(p))
            _bind(lib)
            _motor_lib = lib
            _motor_lib_error = None
            return lib
        except OSError as e:
            last_err = e
    _motor_lib_error = last_err or FileNotFoundError(
        "{0} not found (set AGXMOTOR_LIB or install libmotor)".format(_motor_dynamic_lib_name())
    )
    _motor_lib = None
    return None


def _get_motor_library() -> Optional[CDLL]:
    # 返回进程内缓存的 libmotor，必要时自动加载
    global _motor_lib
    if _motor_lib is not None:
        return _motor_lib
    return _load_motor_library()


def _message_timestamp(msg: Message) -> float:
    # CAN msg timestamp [s], ns precision
    ts = getattr(msg, "timestamp", None)
    if ts is not None:
        return float(ts)
    return time.time_ns() * 1e-9


class Motor:
    """电机控制器。"""

    class MotorParam(Enum):
        """电机参数索引（两字节 ASCII，对应 ``get_param`` / ``set_param``）。

        - ``ID``：关节节点 ID，须为整数值（有效范围 1–15）。
        - ``AC``：轮廓加速度 [rad/s²]，max 12.56。
        - ``DC``：轮廓减速度 [rad/s²]，max 12.56。
        - ``VV``：轮廓速度 [rad/s]，max 20。
        - ``IQ``：力矩环最大电流限制 [A]，max 12。
        - ``OI``：碰撞保护电流阈值 [A]，max 12。
        - ``OT``：碰撞保护时间阈值 [s]，max 2；电流与时间均为 30 时屏蔽碰撞保护。
        - ``TX``：查询/设置是否打开快速反馈帧，0：打开快速帧主动反馈，1：打开快速帧响应式回复（默认0，掉电记忆）。
        - ``TF``：MIT 前馈力矩限幅 [N·m]，对应 ``t_am``，default ±8。
        - ``SO``：关节电机零点 [rad]。
        - ``PP``：位置环比例增益 Kp。
        - ``KP``：速度环比例增益 Kp。
        - ``KI``：速度环积分增益 Ki。
        """

        ID = (ord("i"), ord("d"))
        AC = (ord("a"), ord("c"))
        DC = (ord("d"), ord("c"))
        VV = (ord("v"), ord("v"))
        IQ = (ord("i"), ord("q"))
        OI = (ord("o"), ord("i"))
        OT = (ord("o"), ord("t"))
        TX = (ord("t"), ord("x"))
        TF = (ord("t"), ord("f"))
        SO = (ord("s"), ord("o"))
        PP = (ord("p"), ord("p"))
        KP = (ord("k"), ord("p"))
        KI = (ord("k"), ord("i"))

    def __init__(
        self,
        can_comm: Optional[CanComm] = None,
    ) -> None:
        """
        参数
        ----
        can_comm : CanComm, optional
            总线对象。若提供且具备 ``set_callback``，构造完成后自动调用
            :meth:`attach_can_comm` 对接收发。

        异常
        ----
        RuntimeError
            无法加载共享库，或 ``motor_create`` 返回空句柄。
        """
        self._lib: Optional[CDLL] = None
        self._handle = None
        self._tx_cb: Optional[TxCallback] = None
        self._can_comm: Optional[CanComm] = None

        lib = _get_motor_library()
        if lib is None:
            raise RuntimeError("libmotor not available: {0}".format(_motor_library_load_error() or "unknown error"))
        self._lib = lib
        self._handle = lib.motor_create()
        if not self._handle:
            raise RuntimeError("motor_create failed")
        if can_comm is not None and hasattr(can_comm, "set_callback"):
            self.attach_can_comm(can_comm)

    def _make_tx_callback(self, can_comm: CanComm) -> TxCallback:
        # 构造经 CanComm.send 发帧的 tx callback
        @TX_FN
        def tx(_ctx, id, data_ptr, dlc):
            try:
                payload = bytes(data_ptr[i] for i in range(int(dlc)))
                msg = Message(arbitration_id=int(id), data=payload, is_extended_id=False)
                can_comm.send(msg)
                return 1
            except Exception:
                return 0

        return tx

    def _on_can_frame(self, msg: Message) -> None:
        # CanComm 收帧回调，转发至 handle_rx_once
        self.handle_rx_once(msg=msg)

    def set_tx_callback(self, fn: TxCallback, ctx: Optional[Any] = None) -> None:
        """
        注册底层 CAN 发送回调（不使用 :meth:`attach_can_comm` 时的低级接口）。

        参数
        ----
        fn : TxCallback
            ``ctypes`` 回调；签名为 ``(ctx, can_id, data_ptr, dlc) -> int``，非 0 表示发送成功。
        ctx : object, optional
            传给回调的第一个参数，可为 ``None``。
        """
        self._lib.motor_set_tx_callback(self._handle, fn, ctx)
        self._tx_cb = fn

    def attach_can_comm(self, can_comm: CanComm) -> None:
        """
        将 ``CanComm`` 绑定到本实例：注册 tx callback 与收帧回调。

        收帧为被动模式：须由调用方在应用循环中自行调用 ``can_comm.recv()``，
        收到帧后经回调进入 :meth:`handle_rx_once`。

        参数
        ----
        can_comm : CanComm
            已 ``connect()`` 的总线对象。

        异常
        ----
        RuntimeError
            tx callback 注册失败。
        """
        self.set_tx_callback(self._make_tx_callback(can_comm))
        if not self.has_tx_callback():
            raise RuntimeError("failed to attach tx callback")
        self._can_comm = can_comm
        can_comm.set_callback(self._on_can_frame)

    def has_tx_callback(self) -> bool:
        """
        是否已注册底层 tx 发送回调。

        返回值
        ----
        bool
        """
        return bool(self._lib.motor_has_tx_callback(self._handle))

    def close(self) -> None:
        """
        清除 ``CanComm`` 回调并释放 native 句柄（``motor_destroy``）。
        """
        can_comm = self._can_comm
        if can_comm is not None:
            can_comm.clear_callback()
        self._can_comm = None

        if self._handle and self._lib:
            try:
                self._lib.motor_set_tx_callback(self._handle, TX_FN(), None)
            except Exception:
                pass
            try:
                self._lib.motor_destroy(self._handle)
            except Exception:
                pass
            self._handle = None
        self._tx_cb = None

    def handle_rx_once(
        self,
        id: Optional[int] = None,
        data: Optional[Union[bytes, bytearray]] = None,
        dlc: Optional[int] = None,
        msg: Optional[Message] = None,
        timestamp: Optional[float] = None,
    ) -> bool:
        """
        处理一条接收到的报文，更新内部缓存（高速/低速反馈、参数应答、版本等）。

        参数
        ----
        msg : can.message.Message, optional
            总线接收接口返回的消息对象；若提供，则忽略 ``id``/``data``/``dlc``。
        id : int, optional
            报文标识；仅在未提供 ``msg`` 时使用。
        data : bytes or bytearray, optional
            报文数据；仅在未提供 ``msg`` 时使用。
        dlc : int, optional
            数据长度；未提供时默认为 ``len(data)``。
        timestamp : float, optional
            时间戳 [s]，ns precision；未提供时从 ``msg.timestamp`` 或当前时间推导。

        返回值
        ----
        bool
            ``True``：本条报文被识别为支持的类型并已更新内部状态。
            ``False``：非目标类型或已忽略。

        异常
        ----
        ValueError
            未提供 ``msg`` 且 ``id`` 与 ``data`` 不全时。
        """
        if msg is not None:
            id = int(msg.arbitration_id)
            data = msg.data
            dlc = len(msg.data)
            if timestamp is None:
                timestamp = _message_timestamp(msg)
        if id is None or data is None:
            raise ValueError("need msg= or (id, data)")
        n = int(dlc) if dlc is not None else len(data)
        raw = data if isinstance(data, (bytes, bytearray)) else bytes(data)
        arr = (c_uint8 * max(n, 1))(*raw[:n]) if n > 0 else (c_uint8 * 1)()
        ts = c_uint64(_timestamp_to_ns(timestamp) if timestamp is not None else 0)
        return bool(
            self._lib.motor_handle_rx_once(
                self._handle, c_uint32(id), cast(arr, POINTER(c_uint8)), c_uint8(n), ts
            )
        )

    def get_high_speed_feedback(self, node_id: int) -> Optional[HighSpeedFeedback]:
        """
        读取指定节点最近一次成功解析的高速反馈。

        参数
        ----
        node_id : int
            节点编号（1–15）。

        返回值
        ----
        HighSpeedFeedback | None
            有缓存时返回，无数据为 None。
        """
        out = _MotorHighSpeedFeedback()
        if not self._lib.motor_get_high_speed_feedback(self._handle, node_id, byref(out)):
            return None
        return _high_speed_from_ctypes(out)

    def get_low_speed_feedback(self, node_id: int) -> Optional[LowSpeedFeedback]:
        """
        读取指定节点最近一次成功解析的低速补充反馈。

        参数
        ----
        node_id : int
            节点编号（1–15）。

        返回值
        ----
        LowSpeedFeedback | None
            有缓存时返回，无数据为 None。
        """
        out = _MotorLowSpeedFeedback()
        if not self._lib.motor_get_low_speed_feedback(self._handle, node_id, byref(out)):
            return None
        return _low_speed_from_ctypes(out)

    def _set_target(self, node_id: int, value: float, mode: int, timeout: float = 0.0) -> bool:
        return bool(self._lib.motor_set_target(self._handle, node_id, value, mode, timeout))

    def set_target_velocity(self, node_id: int, velocity: float, timeout: float = 0.0) -> bool:
        """
        下发速度模式目标。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        velocity : float
            目标角速度 [rad/s]。
        timeout : float, optional
            目标指令超时 [s]；0 表示不启用。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return self._set_target(node_id, velocity, 0, timeout)

    def set_target_position(self, node_id: int, position: float, timeout: float = 0.0) -> bool:
        """
        下发位置速度模式目标。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        position : float
            目标关节位置 [rad]。
        timeout : float, optional
            目标指令超时 [s]；0 表示不启用。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return self._set_target(node_id, position, 1, timeout)

    def set_target_current(self, node_id: int, current: float, timeout: float = 0.0) -> bool:
        """
        下发力矩电流模式目标。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        current : float
            目标相电流 [A]。
        timeout : float, optional
            目标指令超时 [s]；0 表示不启用。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return self._set_target(node_id, current, 2, timeout)

    def set_enable(self, node_id: int, enable: bool = True, has_brake: bool = False, brake_on: bool = False) -> bool:
        """
        使能或关闭驱动器输出；可选带抱闸控制。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        enable : bool
            ``True`` 为使能，``False`` 为关闭。
        has_brake : bool
            是否同时控制抱闸；无抱闸机构时置 ``False``。
        brake_on : bool
            抱闸吸合/释放意图；仅 ``has_brake=True`` 时有效。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return bool(
            self._lib.motor_set_enable(
                self._handle, node_id, int(enable), int(has_brake), int(brake_on)
            )
        )

    def set_reset(self, node_id: int, mode: int = 0, timeout: float = 1.0) -> bool:
        """
        驱动器复位或进入/保存标定流程。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        mode : int, optional
            ``0``：普通复位；
            ``1``：进入校准；
            ``2``：保存校准参数（保存后建议再执行一次普通复位）。
        timeout : float, optional
            阻塞应答时间 [s]。

        返回值
        ----
        bool
            应答成功为 True，超时或失败为 False。
        """
        out_ok = c_int()
        if not self._lib.motor_set_reset(
            self._handle, node_id, int(mode), float(timeout), byref(out_ok)
        ):
            return False
        if timeout > 0:
            return bool(out_ok.value)
        return True

    def set_clear_error(self, node_id: int, timeout: float = 1.0) -> bool:
        """
        清除所有故障/告警状态。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        timeout : float, optional
            阻塞应答时间 [s]。

        返回值
        ----
        bool
            应答成功为 True，超时或失败为 False。
        """
        out_ok = c_int()
        if not self._lib.motor_set_clear_error(
            self._handle, node_id, 0, float(timeout), byref(out_ok)
        ):
            return False
        if timeout > 0:
            return bool(out_ok.value)
        return True

    def set_zero_offset(
        self, node_id: int, zero_offset_rad: float, save_to_flash: bool, timeout: float = 1.0
    ) -> bool:
        """
        设置机械零点偏移。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        zero_offset_rad : float
            零点偏移 [rad]。
        save_to_flash : bool
            是否写入非易失存储。
        timeout : float, optional
            阻塞应答时间 [s]。

        返回值
        ----
        bool
            应答成功为 True，超时或失败为 False。
        """
        out_ok = c_int()
        if not self._lib.motor_set_zero_offset(
            self._handle,
            node_id,
            float(zero_offset_rad),
            int(save_to_flash),
            float(timeout),
            byref(out_ok),
        ):
            return False
        if timeout > 0:
            return bool(out_ok.value)
        return True

    def set_profile_acc_dec(self, node_id: int, acceleration: float, deceleration: float) -> bool:
        """
        设置轮廓加速度与减速度。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        acceleration, deceleration : float
            [rad/s²]。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return bool(
            self._lib.motor_set_profile_acc_dec(self._handle, node_id, acceleration, deceleration)
        )

    def set_profile_vel(self, node_id: int, velocity: float) -> bool:
        """
        设置轮廓速度上限。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        velocity : float
            [rad/s]。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return bool(self._lib.motor_set_profile_vel(self._handle, node_id, velocity))

    def set_current_limit(self, node_id: int, current: float) -> bool:
        """
        设置相电流限制。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        current : float
            [A]。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return bool(self._lib.motor_set_current_limit(self._handle, node_id, current))

    def set_mit_control(
        self, node_id: int, p_des: float, v_des: float, kp: float, kd: float, t_ff: float, t_am: float = 16.0
    ) -> bool:
        """
        MIT 模式控制量下发（无附加校验字段的版本）。

        下发前各量会按实现侧区间做饱和映射；超出区间时按边界钳位。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        p_des : float
            期望关节位置 [rad]；有效映射区间 [-12.5, 12.5]。
        v_des : float
            期望关节速度 [rad/s]；有效映射区间 [-45.0, 45.0]。
        kp : float
            位置环比例增益；有效映射区间 [0.0, 500.0]。
        kd : float
            微分相关增益；有效映射区间 [-5.0, 5.0]。
        t_ff : float
            前馈力矩 [N·m]；线性映射到对称区间 [-t_am, +t_am]（见参数 ``t_am``）。
        t_am : float, optional
            定义 ``t_ff`` 映射半幅 [N·m]，即允许范围 [-t_am, +t_am]；默认 16.0。
            若 ``<= 0``，实现侧按 16.0 处理。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。

        说明
        ----
        部分固件要求控制量带校验字段；此种情况下应使用 ``set_mit_control_crc`` 而非本方法。
        两种接口通常只需选用其一。
        """
        return bool(
            self._lib.motor_set_mit_control(
                self._handle, node_id, p_des, v_des, kp, kd, t_ff, float(t_am)
            )
        )

    def set_mit_control_crc(
        self, node_id: int, p_des: float, v_des: float, kp: float, kd: float, t_ff: float, t_am: float = 8.0
    ) -> bool:
        """
        MIT 模式控制量下发（带校验字段的格式）。

        下发前各量均会做饱和映射；超出有效区间时按边界钳位。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        p_des : float
            期望关节位置 [rad]；有效映射区间 [-12.5, 12.5]。
        v_des : float
            期望关节速度 [rad/s]；有效映射区间 [-45.0, 45.0]。
        kp : float
            位置环比例增益；有效映射区间 [0.0, 500.0]。
        kd : float
            微分相关增益；有效映射区间 [-5.0, 5.0]。
        t_ff : float
            前馈力矩 [N·m]；线性映射到对称区间 [-t_am, +t_am]（见参数 ``t_am``）。
        t_am : float, optional
            定义 ``t_ff`` 映射半幅 [N·m]，即允许范围 [-t_am, +t_am]；默认 8.0。
            若 ``<= 0``，实现侧按 8.0 处理。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。

        说明
        ----
        与无校验版本功能相同，但报文含额外校验字段；具体以现场固件支持情况为准。
        两种 MIT 控制接口通常只需选用其一。
        """
        return bool(
            self._lib.motor_set_mit_control_crc(
                self._handle, node_id, p_des, v_des, kp, kd, t_ff, float(t_am)
            )
        )

    def set_collision_threshold(self, node_id: int, current_threshold: float, time_threshold: float) -> bool:
        """
        设置碰撞保护电流与时间阈值。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        current_threshold : float
            电流阈值 [A]。
        time_threshold : float
            时间阈值 [s]。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return bool(
            self._lib.motor_set_collision_threshold(
                self._handle, node_id, current_threshold, time_threshold
            )
        )

    def get_param(self, node_id: int, param: MotorParam, timeout: float = 1.0) -> Optional[float]:
        """
        查询驱动器参数。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        param : MotorParam
            参数类型，例如 ``Motor.MotorParam.VV`` [rad/s]。
        timeout : float, optional
            阻塞应答时间 [s]。

        返回值
        ----
        float | None
            成功返回参数值，超时或发送失败为 None。
            量纲见 :class:`Motor.MotorParam`。
        """
        idx1, idx2 = param.value
        out = c_float()
        if not self._lib.motor_get_param(self._handle, node_id, idx1, idx2, float(timeout), byref(out)):
            return None
        return float(out.value)

    def set_param(self, node_id: int, param: MotorParam, value: float, timeout: float = 1.0) -> bool:
        """
        设置驱动器参数。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        param : MotorParam
            参数类型，例如 ``Motor.MotorParam.VV`` [rad/s]。
        value : float
            参数值；量纲见 ``Motor.MotorParam``。
        timeout : float, optional
            阻塞应答时间 [s]。

        返回值
        ----
        bool
            写入成功为 True，超时或失败为 False。
        """
        idx1, idx2 = param.value
        return bool(
            self._lib.motor_set_param(self._handle, node_id, idx1, idx2, value, float(timeout))
        )

    def get_version(self, node_id: int, timeout: float = 1.0) -> Optional[VersionInfo]:
        """
        查询驱动器版本信息。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        timeout : float, optional
            阻塞应答时间 [s]。

        返回值
        ----
        VersionInfo | None
            成功返回 VersionInfo，超时或发送失败为 None。
        """
        out = _MotorVersionInfo()
        if not self._lib.motor_get_version(self._handle, node_id, float(timeout), byref(out)):
            return None
        return _version_from_ctypes(out)


__all__ = [
    "DriverStatus",
    "HighSpeedFeedback",
    "LowSpeedFeedback",
    "Motor",
    "VersionInfo",
]
