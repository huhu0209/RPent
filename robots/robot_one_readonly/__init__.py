"""Robot One read-only extension; package import creates no robot resources."""

from robots.robot_one_readonly.robot_spec import get_robot_spec, get_toolkit

__all__ = ["get_robot_spec", "get_toolkit"]
