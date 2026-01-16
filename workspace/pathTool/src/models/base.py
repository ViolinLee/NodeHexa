from __future__ import annotations

from typing import Tuple


class RobotPathModel:
    """路径生成与 C 代码导出抽象接口（与原 main.py 行为保持一致）"""

    def verify_path(self, path_name: str, params: Tuple) -> bool:
        raise NotImplementedError

    def generate_c_body(self, path_name: str, params: Tuple) -> str:
        raise NotImplementedError

    def generate_c_def(self, path_name: str) -> str:
        raise NotImplementedError

