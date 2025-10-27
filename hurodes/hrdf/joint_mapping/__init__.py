from hurodes.hrdf.joint_mapping.base_solver import DirectSolver
from hurodes.hrdf.joint_mapping.double_link_solver import DoubleLinkSolver

solver_dict = {
    "double_link": DoubleLinkSolver,
    "direct": DirectSolver,
}

__all__ = ["solver_dict"]
