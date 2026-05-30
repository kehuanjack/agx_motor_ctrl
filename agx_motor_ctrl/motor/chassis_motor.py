"""底盘电机 Python 封装（ctypes + libmotor_chassis）。"""
from typing import Any, Optional, Union

from ctypes import (
    byref,
    cast,
    c_int,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    POINTER,
)
from can.message import Message

from agx_motor_ctrl.comm.can_comm import CanComm
from ._motor_common import (
    TX_FN,
    TxCallback,
    DriverStatus,
    HighSpeedFeedback,
    LowSpeedFeedback,
    _chassis_cache,
    _MotorHighSpeedFeedback,
    _MotorLowSpeedFeedback,
    high_speed_from_ctypes,
    low_speed_from_ctypes,
    message_timestamp,
    timestamp_to_ns,
)


class ChassisMotor:
    """底盘轮毂/转向电机控制器（``libmotor_chassis``）。

    与 :class:`~agx_motor_ctrl.motor.arm_motor.ArmMotor` 使用不同 CAN 报文格式；同一进程可各建一个实例，
    但同一条 ``CanComm`` 上须由应用层按 ``node_id`` 分发收帧，不宜两个类同时 ``attach_can_comm``。

    量纲约定
    --------
    - 高速反馈 ``position`` [INC]、``velocity`` [RPM]、``current`` [A]
    - 轮廓加速度/减速度 [RPM/s]；轮廓速度上限 [RPM]（线值为 uint16，实现侧钳位）
    - 相电流、母线电流等与 Arm 一致为 [A]
    - 低速 ``status`` 复用通用结构体字段名：``collision_tripped`` 表示传感器异常，
      ``stall_tripped`` 表示已回零；``enabled`` 与 Arm 相同（status 字节 bit6 为 1 表示使能）

    不提供 MIT、参数读写、版本查询、碰撞阈值等 Arm 专有接口。
    """

    def __init__(self, can_comm: Optional[CanComm] = None) -> None:
        """
        参数
        ----
        can_comm : CanComm, optional
            总线对象。若提供且具备 ``set_callback``，构造完成后自动调用
            :meth:`attach_can_comm` 对接收发。

        异常
        ----
        RuntimeError
            无法加载 ``libmotor_chassis``，或 ``motor_create`` 返回空句柄。
        """
        self._lib = None
        self._handle = None
        self._tx_cb: Optional[TxCallback] = None
        self._can_comm: Optional[CanComm] = None

        lib = _chassis_cache.get()
        if lib is None:
            raise RuntimeError(
                "libmotor_chassis not available: {0}".format(
                    _chassis_cache.load_error() or "unknown error"
                )
            )
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
        处理一条接收到的报文，更新内部缓存（高速/低速反馈、简单指令应答等）。

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
            ``True``：本条报文被识别为底盘协议支持的类型并已更新内部状态。
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
                timestamp = message_timestamp(msg)
        if id is None or data is None:
            raise ValueError("need msg= or (id, data)")
        n = int(dlc) if dlc is not None else len(data)
        raw = data if isinstance(data, (bytes, bytearray)) else bytes(data)
        arr = (c_uint8 * max(n, 1))(*raw[:n]) if n > 0 else (c_uint8 * 1)()
        ts = c_uint64(timestamp_to_ns(timestamp) if timestamp is not None else 0)
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
            有缓存时返回；``position`` 为线值 int32 大端解析的编码器增量 [INC]（有符号），
            ``velocity`` [RPM]、``current`` [A]、``timestamp`` [s]。无数据为 None。
        """
        out = _MotorHighSpeedFeedback()
        if not self._lib.motor_get_high_speed_feedback(self._handle, node_id, byref(out)):
            return None
        return high_speed_from_ctypes(out)

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
            有缓存时返回。``status.enabled`` 与 Arm 一致（bit6=1 为使能）；
            ``collision_tripped`` 为传感器异常，``stall_tripped`` 为回零完成。无数据为 None。
        """
        out = _MotorLowSpeedFeedback()
        if not self._lib.motor_get_low_speed_feedback(self._handle, node_id, byref(out)):
            return None
        return low_speed_from_ctypes(out)

    def _set_target(self, node_id: int, value: float, mode: int, timeout: float = 0.0) -> bool:
        return bool(self._lib.motor_set_target(self._handle, node_id, value, mode, timeout))

    def set_target_rpm(self, node_id: int, rpm: float, timeout: float = 0.0) -> bool:
        """
        下发转速模式目标（``0x410`` mode 0）。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        rpm : float
            目标转速 [RPM]，线值为 int32 大端。
        timeout : float, optional
            目标指令超时 [s]；0 表示不启用。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return self._set_target(node_id, rpm, 0, timeout)

    def set_target_inc(self, node_id: int, inc: float, timeout: float = 0.0) -> bool:
        """
        下发位置模式目标（``0x410`` mode 1）。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        inc : float
            目标编码器增量 [INC]，线值为 int32 大端。
        timeout : float, optional
            目标指令超时 [s]；0 表示不启用。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return self._set_target(node_id, inc, 1, timeout)

    def set_target_current(self, node_id: int, current: float, timeout: float = 0.0) -> bool:
        """
        下发电流模式目标（``0x410`` mode 2）。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        current : float
            目标相电流 [A]（内部换算为 mA 线值）。
        timeout : float, optional
            目标指令超时 [s]；0 表示不启用。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return self._set_target(node_id, current, 2, timeout)

    def set_enable(
        self, node_id: int, enable: bool = True, has_brake: bool = False, brake_on: bool = False
    ) -> bool:
        """
        使能或关闭驱动器输出；可选带抱闸控制（``0x420``）。

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
        驱动器复位或进入/保存标定流程（``0x000``）。

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
        清除所有故障/告警状态（``0x010``）。

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
        self,
        node_id: int,
        zero_offset: float = 0.0,
        save_to_flash: bool = False,
        timeout: float = 1.0,
    ) -> bool:
        """
        设置编码器零点偏移（``0x020``）。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        zero_offset : float, optional
            零点偏移 [INC]；默认 ``0.0`` 表示在当前位置标零、不附加增量偏移。
        save_to_flash : bool, optional
            是否写入非易失存储；默认 ``False``。
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
            float(zero_offset),
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
        设置轮廓加速度与减速度（``0x430``）。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        acceleration, deceleration : float
            轮廓加速度、减速度 [RPM/s]（uint16 大端，实现侧钳位）。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return bool(
            self._lib.motor_set_profile_acc_dec(self._handle, node_id, acceleration, deceleration)
        )

    def set_profile_vel(self, node_id: int, rpm: float) -> bool:
        """
        设置轮廓速度上限（``0x440``）。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        rpm : float
            轮廓速度上限 [RPM]（uint16 大端，实现侧钳位）。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return bool(self._lib.motor_set_profile_vel(self._handle, node_id, rpm))

    def set_current_limit(self, node_id: int, current: float) -> bool:
        """
        设置相电流限制（``0x450``）。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        current : float
            电流上限 [A]。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return bool(self._lib.motor_set_current_limit(self._handle, node_id, current))

    def set_stiffness(self, node_id: int, stiffness: int) -> bool:
        """
        设置底盘刚度（``0x470``，底盘专有）。

        参数
        ----
        node_id : int
            节点编号（1–15）。
        stiffness : int
            刚度线值（uint16 大端）；实现侧钳位到 10–2000。

        返回值
        ----
        bool
            成功发送为 True，失败为 False。
        """
        return bool(
            self._lib.motor_set_stiffness(self._handle, node_id, c_uint16(int(stiffness)))
        )


__all__ = [
    "ChassisMotor",
    "DriverStatus",
    "HighSpeedFeedback",
    "LowSpeedFeedback",
]
