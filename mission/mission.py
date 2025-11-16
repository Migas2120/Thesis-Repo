from typing import List, Union, Tuple, Optional
from dataclasses import dataclass, field


class Mission:
    def __init__(
        self,
        mission_id: str,
        waypoints: List[Union[Tuple[float, float, float], dict]],
        mode: str = "once",             # Add default mode
        patrol_loops: int = 1,
        start_index=0,                      # Add default patrol loop count
        start_time: Optional[float] = None  # NEW
    ):
        self.mission_id = mission_id
        self.executed = False
        self.current_wp_index = 0
        self.last_command_time = None
        self.last_distance = None
        self.retry_count = 0
        self.start_index = start_index
        self.mode = mode  # "once" (default) or "patrol"
        self.patrol_loops = patrol_loops
        self.completed_loops = 0

        self.waypoints = []
        for wp in waypoints:
            if isinstance(wp, dict) and "coords" in wp:
                # Already in extended format
                self.waypoints.append({
                    "coords": tuple(wp["coords"]),
                    "hold": wp.get("hold", 0)
                })
            elif isinstance(wp, (list, tuple)) and len(wp) == 3:
                # Basic format — convert to dict
                self.waypoints.append({
                    "coords": tuple(wp),
                    "hold_duration": 0
                })
            else:
                raise ValueError("Waypoints must be [x, y, z] or {coords: [x, y, z], hold_duration: seconds}")
        
    def pause(self, current_index):
        return Mission(
            mission_id=self.mission_id,
            waypoints=self.waypoints,
            mode=self.mode,
            patrol_loops=self.patrol_loops,
            start_index=current_index
        )




@dataclass(order=True)
class PrioritizedMission:
    priority: int
    mission: Mission = field(compare=False)
    mission_id: str = field(compare=False)
    valid: bool = field(default=True, compare=False)