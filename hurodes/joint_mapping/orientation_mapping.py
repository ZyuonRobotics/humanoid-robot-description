import numpy as np
from typing import Any

from pydantic import Field
from hurodes.utils.config import BaseConfig

class OrientationMappingConfig(BaseConfig):
    model_config = {"arbitrary_types_allowed": True}
    
    motor_id_list: list[int] = []
    negative_list: list[int] = []
    # nx1 matrix
    mapping_matrix: np.ndarray = Field(default=None, init=False, exclude=True)

    def model_post_init(self, __context: Any) -> None:
        assert len(self.motor_id_list) == len(set(self.motor_id_list)), f"Motor ID list must be unique: {self.motor_id_list}"
        assert len(self.negative_list) == len(set(self.negative_list)), f"Negative list must be unique: {self.negative_list}"
        self._generate_mapping_matrix()

    def _generate_mapping_matrix(self):
        self.mapping_matrix = np.ones(len(self.motor_id_list))
        self.mapping_matrix[self.negative_list] = -1

    def pvt_transform(self, joint_pvt: np.ndarray):
        assert len(joint_pvt) == len(self.motor_id_list), f"Joint PVT must have {len(self.motor_id_list)}, got {len(joint_pvt)}."
        return self.mapping_matrix * joint_pvt

if __name__ == "__main__":
    from hurodes import ROBOTS_PATH

    config = OrientationMappingConfig.from_yaml(ROBOTS_PATH / "zhaplin-10dof" / "joint_mapping.yaml")
    print(config)
    pvt = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0])
    print(config.pvt_transform(pvt))
