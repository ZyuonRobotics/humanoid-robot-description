import pytest
from hurodes.mjcf_parser.without_waist_parser import WithoutWaistMJCFParser

def test_without_waist_parser_init(tmp_path):
    # Create a minimal MJCF XML file for testing
    xml_content = """
    <mujoco>
        <worldbody>
            <body name='base'>
                <body name='leg'/>
                <body name='arm'/>
            </body>
        </worldbody>
    </mujoco>
    """
    xml_path = tmp_path / "test_waist.xml"
    xml_path.write_text(xml_content)
    parser = WithoutWaistMJCFParser(str(xml_path))
    assert parser.mjcf_path == str(xml_path)
    assert parser.base_link is not None
    assert hasattr(parser, 'body_branches')

def test_without_waist_parser_parse_prints_names(tmp_path, capsys):
    xml_content = """
    <mujoco>
        <worldbody>
            <body name='base'>
                <body name='leg'/>
                <body name='arm'/>
            </body>
        </worldbody>
    </mujoco>
    """
    xml_path = tmp_path / "test_waist.xml"
    xml_path.write_text(xml_content)
    parser = WithoutWaistMJCFParser(str(xml_path))
    parser.parse()
    captured = capsys.readouterr()
    assert "leg" in captured.out
    assert "arm" in captured.out 