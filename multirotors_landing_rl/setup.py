from setuptools import setup, find_packages

setup(
    name='multirotors_landing_rl',
    version='0.0.1',
    author='Luca Bartolomei',
    author_email='lbartolomei@ethz.ch',
    description='RL library to train an agent to land a multicopter safely',
    long_description='',
    install_requires=['mlgym', 'seaborn', 'matplotlib'],
    packages=[package for package in find_packages()
              if package.startswith('multirotors_landing_rl')]
)
