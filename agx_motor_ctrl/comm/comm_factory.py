from typing import Dict, Type, TypeVar

from .can_comm import CanComm, create_can_comm_config

T = TypeVar("T")


def create_comm_config(comm: str = "can", **kwargs) -> dict:
    if comm == "can":
        return create_can_comm_config(**kwargs)
    raise ValueError(f"Unsupported comm type: {comm}")


class CommsFactory:

    _registry: Dict[str, Dict[str, Type]] = {
        "can": {"impl": CanComm},
    }

    @classmethod
    def register_comm(cls, type: str, version: str, comm_cls: Type) -> None:
        cls._registry.setdefault(type, {})[version] = comm_cls

    @classmethod
    def load_class(cls, type: str, version: str = "impl") -> Type:
        return cls._registry[type][version]

    @classmethod
    def create_comm(cls, type: str, version: str = "impl", **kwargs) -> T:
        cls_type = cls._registry[type][version]
        return cls_type(**kwargs)
