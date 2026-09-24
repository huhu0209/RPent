from __future__ import annotations

import os
from pathlib import Path
from typing import Any
from xml.etree import ElementTree


DEFAULT_TREE_NAME = "lynsense_single_box_tree.xml"
NODE_NAME = "lynsense_single_box_tree"


def create_root(tree_path: Path | str | None = None) -> Any:
    import py_trees

    from lynsense_webots_sim.tree_behaviours import (
        EnsureSimulationManipulationServices,
        SimulationMoveDistance,
        SimulationNavToPose,
        SimulationPlanBoxPhase,
        SimulationPlanMoveNamedConfig,
        SimulationPlanMoveWaist,
        SimulationPlanSetGripper,
    )

    source = Path(tree_path or _default_tree_path())
    root = ElementTree.parse(source).getroot()
    if root.tag != "py_trees.composites.Sequence" or root.attrib.get("memory") != "True":
        raise ValueError("single-box tree root must be a memory Sequence")

    factories = {
        "EnsureSimulationManipulationServices": lambda attrs: (
            EnsureSimulationManipulationServices(name=attrs["name"])
        ),
        "SimulationNavToPose": lambda attrs: SimulationNavToPose(
            goal_name=attrs["goal_name"], name=attrs["name"]
        ),
        "SimulationMoveDistance": lambda attrs: SimulationMoveDistance(
            distance=float(attrs["distance"]),
            angle=float(attrs["angle"]),
            name=attrs["name"],
        ),
        "SimulationPlanMoveWaist": lambda attrs: SimulationPlanMoveWaist(
            height_mm=float(attrs["height_mm"]), name=attrs["name"]
        ),
        "SimulationPlanMoveNamedConfig": lambda attrs: SimulationPlanMoveNamedConfig(
            target=attrs["target"], name=attrs["name"]
        ),
        "SimulationPlanSetGripper": lambda attrs: SimulationPlanSetGripper(
            position=float(attrs["position"]), name=attrs["name"]
        ),
        "SimulationPlanBoxPhase": lambda attrs: SimulationPlanBoxPhase(
            action=attrs["action"],
            flow=attrs["flow"],
            config=attrs["config"],
            name=attrs["name"],
        ),
    }
    sequence = py_trees.composites.Sequence(root.attrib["name"], memory=True)
    for element in root:
        if element.tag not in factories:
            raise ValueError(f"unsupported single-box tree node: {element.tag}")
        sequence.add_child(factories[element.tag](element.attrib))
    return sequence


def main(args: Any = None) -> int:
    import rclpy
    from rclpy.node import Node
    import py_trees

    rclpy.init(args=args)
    node: Node | None = None
    tree: Any = None
    root: Any = None
    try:
        node = Node(NODE_NAME)
        import py_trees_ros

        root = create_root()
        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=False)
        tree.setup(node=node)

        def stop_after_success(ticked_tree: Any) -> None:
            if ticked_tree.root.status == py_trees.common.Status.SUCCESS:
                ticked_tree.stop()

        tree.add_post_tick_handler(stop_after_success)
        tree.tick_tock(period_ms=20)
        while rclpy.ok() and root.status != py_trees.common.Status.SUCCESS:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if tree is not None and hasattr(tree, "stop"):
            tree.stop()
        if root is not None:
            _shutdown_behaviours(root)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _default_tree_path() -> Path:
    configured = os.environ.get("LYNSENSE_SINGLE_BOX_TREE")
    if configured:
        return Path(configured)
    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory("lynsense_webots_sim"))
        / "trees"
        / DEFAULT_TREE_NAME
    )


def _shutdown_behaviours(root: Any) -> None:
    shutdown = getattr(root, "shutdown", None)
    if shutdown is not None:
        shutdown()
    for child in getattr(root, "children", ()):
        _shutdown_behaviours(child)


if __name__ == "__main__":
    raise SystemExit(main())
