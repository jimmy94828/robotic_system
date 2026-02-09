from dataclasses import dataclass
from typing import Optional
from geometry_msgs.msg import PoseStamped

@dataclass
class Command:
    """Một lệnh trừu tượng để executor xử lý."""
    kind: str                          # 'NAV' | 'GRASP' | ...
    target_pose: Optional[PoseStamped] = None
    object_id: Optional[str] = None
    meta: Optional[dict] = None        # thêm tham số khác nếu cần
