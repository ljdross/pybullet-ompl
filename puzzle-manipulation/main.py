import sys

from src.manipulation import Manipulation
from src.config.config_parser import ConfigParser
from src import obs


def main():
    config_json = sys.argv[1] if len(sys.argv) > 1 else './configurations/simple_sliders_config.json'
    defaults_json = sys.argv[2] if len(sys.argv) > 2 else './configurations/defaults.json'

    config_parser = ConfigParser(config_json, defaults_json)

    config = config_parser.parse()

    manipulation = Manipulation(config)
    manipulation.plan()

    obs.start_recording()
    manipulation.execute()
    obs.stop_recording()


if __name__ == '__main__':
    main()
