from hurodes.mjcf_generator.unified_generator import UnifiedMJCFGenerator, dict2str, find_by_body_id
from hurodes import ROBOTS_PATH

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
    # Real tests would require a valid hrdf_path structure
    from hurodes.mjcf_generator.unified_generator import UnifiedMJCFGenerator
    generator = UnifiedMJCFGenerator(hrdf_path=tmp_path)
    assert generator.hrdf_path == tmp_path
    assert generator.disable_gravity is False
    assert generator.time_step == 0.001
