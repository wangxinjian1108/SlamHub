from scripts.registration.base import RegistrationBase, RegistrationResult

_METHODS = {}


def register_method(name):
    def decorator(cls):
        _METHODS[name] = cls
        return cls
    return decorator


def get_registration_method(name: str, **kwargs) -> RegistrationBase:
    if not _METHODS:
        import scripts.registration.icp  # noqa: F401
    if name not in _METHODS:
        available = ", ".join(_METHODS.keys())
        raise ValueError(f"Unknown method '{name}'. Available: {available}")
    return _METHODS[name](**kwargs)
