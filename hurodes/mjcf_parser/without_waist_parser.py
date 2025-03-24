import os

from hurodes.mjcf_parser.base_parser import BaseMJCFParser
from hurodes.contants import RobotFormatType

class WithoutWaistMJCFParser(BaseMJCFParser):
    format_type = RobotFormatType.WITHOUT_WAIST

    def __init__(self, mjcf_path):
        super(WithoutWaistMJCFParser, self).__init__(mjcf_path)

        self.body_branches = {""}

    def parse(self):
        names = [body.get("name") for body in self.base_link.findall('body')]
        print(names)

    def save(self, save_path=None):
        super(WithoutWaistMJCFParser, self).save(save_path)



if __name__ == '__main__':
    from hurodes import MJCF_ROBOTS_PATH

    mjcf_path = os.path.join(MJCF_ROBOTS_PATH, "kuavo_s45", "mjcf", "biped_s45.xml")
    parser = WithoutWaistMJCFParser(mjcf_path)

    parser.print_body_tree()

    body_tree, bodies_data = parser.parse()
    print(body_tree)
    print(bodies_data)