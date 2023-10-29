from src.manipulation import Manipulation
from src.configurations import challenges, manipulation_arguments


def main():
    config = challenges.simple_sliders
    parameters = manipulation_arguments.defaults

    manipulation = Manipulation(config, parameters)
    manipulation.plan()
    manipulation.execute()


if __name__ == '__main__':
    main()
