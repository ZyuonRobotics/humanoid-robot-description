import os
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd
from pathlib import Path
from hurodes.mjcf_generator.unified_generator import UnifiedMJCFGenerator, dict2str, find_by_body_id
from hurodes.contants import RobotFormatType
from hurodes import ROBOTS_PATH


def test_dict2str():
    data = {
        "a0": 1, "a1": 2, "a2": 3,
        "b": 1.1,
        "c0": 1.1, "c1": 1.2, "c2": 1.3
    }
    assert dict2str(data, "a") == "1 2 3"
    assert dict2str(data, "b") == "1.1"
    assert dict2str(data, "c") == "1.1 1.2 1.3"

def test_find_by_body_id():
    all_data = [
        {"bodyid": 0, "name": "root"},
        {"bodyid": 1, "name": "arm"},
        {"bodyid": 0, "name": "root2"},
    ]
    res = find_by_body_id(all_data, 0)
    assert len(res) == 2
    assert all("bodyid" not in d for d in res)
    assert set(d["name"] for d in res) == {"root", "root2"}

def test_unified_mjcf_generator_init(tmp_path):
    # This is a placeholder for UnifiedMJCFGenerator tests
    # Real tests would require a valid ehdf_path structure
    from hurodes.mjcf_generator.unified_generator import UnifiedMJCFGenerator
    generator = UnifiedMJCFGenerator(ehdf_path=tmp_path)
    assert generator.ehdf_path == tmp_path
    assert generator.disable_gravity is False
    assert generator.time_step == 0.001
