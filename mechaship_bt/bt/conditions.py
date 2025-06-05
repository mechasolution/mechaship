from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access, Status


class IsWaypointReached(Behaviour):
    def __init__(self) -> None:
        super().__init__(name="IsWaypointReached")
        self.bb = Client(name="Mechaship")
        self.bb.register_key(key="waypoint_reached", access=Access.READ)

    def update(self) -> Status:
        return Status.SUCCESS if self.bb.get("waypoint_reached") else Status.FAILURE


class IsMarkerDetected(Behaviour):
    def __init__(self) -> None:
        super().__init__(name="IsMarkerDetected")
        self.bb = Client(name="Mechaship")
        self.bb.register_key(key="marker_detected", access=Access.READ)

    def update(self) -> Status:
        return Status.SUCCESS if self.bb.get("marker_detected") else Status.FAILURE
