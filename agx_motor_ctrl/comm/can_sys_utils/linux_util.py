import os
import subprocess
from typing import List, Optional

from .base_util import CanSystemInfoBase


class LinuxSocketCanSystemInfo(CanSystemInfoBase):

    @staticmethod
    def is_exists(channel: str) -> bool:
        return os.path.exists(f"/sys/class/net/{channel}")

    @staticmethod
    def is_up(channel: str) -> bool:
        oper_path = f"/sys/class/net/{channel}/operstate"
        if not os.path.exists(oper_path):
            return False
        with open(oper_path, "r") as f:
            state = f.read().strip()
        if state == "up":
            return True
        if state == "unknown":
            flags_path = f"/sys/class/net/{channel}/flags"
            try:
                with open(flags_path, "r") as ff:
                    flags = int(ff.read().strip(), 16)
                return (flags & 0x1) != 0
            except (OSError, ValueError):
                return False
        return False

    @staticmethod
    def get_bitrate(channel: str) -> Optional[int]:
        try:
            result = subprocess.run(
                ["ip", "-details", "link", "show", channel],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                check=False,
            )
            if result.returncode != 0:
                return None
            for line in result.stdout.split("\n"):
                if "bitrate" in line:
                    return int(line.split("bitrate ")[1].split(" ")[0])
        except (ValueError, IndexError, OSError):
            return None
        return None

    @staticmethod
    def get_available_can_channel() -> List[str]:
        return [item for item in os.listdir("/sys/class/net/") if "can" in item]
