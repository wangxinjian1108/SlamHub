from .base import RegistrationBase, RegistrationResult

_METHODS = {}


def register_method(name):
    def decorator(cls):
        _METHODS[name] = cls
        return cls
    return decorator


def _ensure_methods_loaded():
    if not _METHODS:
        from . import icp  # noqa: F401
        from . import icp_pl  # noqa: F401


def get_registration_method(name: str, **kwargs) -> RegistrationBase:
    _ensure_methods_loaded()
    if name not in _METHODS:
        available = ", ".join(_METHODS.keys())
        raise ValueError(f"Unknown method '{name}'. Available: {available}")
    return _METHODS[name](**kwargs)
