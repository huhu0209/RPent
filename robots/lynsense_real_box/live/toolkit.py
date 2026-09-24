"""Live-only Toolkit wrapper constructed by the reviewed composition root."""

from __future__ import annotations

from threading import Lock

from typing import Any

from rpent.dashboard.events import DashboardEventSink
from rpent.memory import MemoryManager

from robots.lynsense_real_box.atomic_adapter import LynrotControlAtomicAdapter
from robots.lynsense_real_box.atomic_profile import AtomicCapabilityProfile
from robots.lynsense_real_box.atomic_toolkit import LynsenseAtomicToolkit
from robots.lynsense_real_box.evidence import EvidenceRecorder


class LynsenseLiveAtomicToolkit(LynsenseAtomicToolkit):
    """Close all live-owned resources, not only the transport-free coordinator."""

    _allows_live_dispatch = True

    def __init__(
        self,
        *,
        adapter: LynrotControlAtomicAdapter,
        perception: Any,
        perception_identity: Any,
        chassis_transport: Any,
        owned_runtime: Any,
        evidence: EvidenceRecorder,
        dashboard_events: DashboardEventSink,
        memory: MemoryManager,
        profile: AtomicCapabilityProfile,
    ) -> None:
        super().__init__(
            adapter=adapter,
            perception=perception,
            perception_identity=perception_identity,
            evidence=evidence,
            dashboard_events=dashboard_events,
            memory=memory,
            profile=profile,
        )
        self._chassis_transport = chassis_transport
        self._owned_runtime = owned_runtime
        self._close_lock = Lock()
        self._closed_resources: set[str] = set()
        self._owned_release_failure: str | None = None

    def close(self) -> None:
        """Close transports before adapter ownership, runtime claim, and evidence."""

        with self._close_lock:
            with self._operation_lock:
                if self._closed:
                    return
                if self._active_operation is not None:
                    raise RuntimeError(
                        "cannot close while a tool operation is active"
                    )
                if self._adapter.capability_mode in {
                    "chassis_moving",
                    "waist_moving",
                    "dual_arms_moving",
                    "grippers_moving",
                    "stopping",
                }:
                    raise RuntimeError(
                        "cannot close while an atomic capability is active; request "
                        "and confirm stop first"
                    )
                errors: list[BaseException] = []
                resources = (
                    ("perception", self._perception.close),
                    ("chassis", self._chassis_transport.close),
                    ("adapter", self._adapter.close),
                    ("owned_runtime", self._owned_runtime.release),
                    ("evidence", self._evidence.close),
                )
                for name, close in resources:
                    if name in self._closed_resources:
                        continue
                    try:
                        close()
                    except BaseException as error:
                        errors.append(error)
                        if name == "owned_runtime":
                            self._closed_resources.add(name)
                            self._owned_release_failure = str(error)
                    else:
                        self._closed_resources.add(name)
                if errors:
                    raise RuntimeError(
                        "live atomic Toolkit close failed: "
                        + "; ".join(str(error) for error in errors)
                    )
                if self._owned_release_failure is not None:
                    raise RuntimeError(
                        "live atomic Toolkit close failed: owned runtime "
                        f"release is unresolved: {self._owned_release_failure}"
                    )
                self._closed = True
