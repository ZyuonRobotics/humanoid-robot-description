import os

from hurodes.mjcf_parser.unified_parser import UnifiedMJCFParser
from hurodes.contants import RobotFormatType

class WithoutWaistMJCFParser(UnifiedMJCFParser):
    format_type = RobotFormatType.WITHOUT_WAIST

    def __init__(self, mjcf_path):
        super(WithoutWaistMJCFParser, self).__init__(mjcf_path)

        self.body_branches = {""}

    def parse(self):
        names = [body.get("name") for body in self.base_link.findall('body')]
        print(names)

    def save(self, save_path=None):
        super(WithoutWaistMJCFParser, self).save(save_path)
