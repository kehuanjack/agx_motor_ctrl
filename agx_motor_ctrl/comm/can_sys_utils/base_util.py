from abc import ABC, abstractmethod
from typing import Optional


class CanSystemInfoBase(ABC):

    @staticmethod
    @abstractmethod
    def is_exists(channel: str) -> bool:
        ...

    @staticmethod
    @abstractmethod
    def is_up(channel: str) -> bool:
        ...

    @staticmethod
    @abstractmethod
    def get_bitrate(channel: str) -> Optional[int]:
        ...
