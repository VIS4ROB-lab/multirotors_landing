import os
import glob
import shutil
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        ML_EXTERNAL_FILES = os.environ["ML_PATH"] + \
            "/multirotors_landing_lib/externals/"
        # --------------------------------
        # remove cached external files
        # a hack to solve some cmake error when using "pip install ."
        try:
            for i, p in enumerate(glob.glob(os.path.join(ML_EXTERNAL_FILES, "*"))):
                shutil.rmtree(p)
                print("Removing some cache file: ", p)
        except:
            pass
        ML_BUILD_FILES = os.environ["ML_PATH"] + \
            "/multirotors_landing_lib/build/"
        # --------------------------------
        # remove cached build files
        # a hack to solve some cmake error when using "pip install ."
        try:
            for i, p in enumerate(glob.glob(os.path.join(ML_BUILD_FILES, "*"))):
                shutil.rmtree(p)
                print("Removing some cache file: ", p)
        except:
            pass
        # --------------------------------

        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(
            self.get_ext_fullpath(ext.name)))
        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Release'
        build_args = ['--config', cfg]
        
        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        cmake_args += ['-DPATH_PROJECT=' + os.environ["ML_PATH"] + '/..']
        build_args += ['--', '-j8']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] +
                              cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] +
                              build_args, cwd=self.build_temp)


setup(
    name='mlgym',
    version='0.0.1',
    author='Luca Bartolomei',
    author_email='lbartolomei@ethz.ch',
    description='Multirotors Landing Gym Environment',
    long_description='',
    ext_modules=[CMakeExtension('multirotors_landing_lib')],
    install_requires=['gym==0.11', 'ruamel.yaml',
                      'numpy', 'stable_baselines==2.10.1',
                      'cloudpickle==1.6.0', 'tensorflow==1.14',
                      'tensorflow-gpu==1.14',
                      'tensorflow-probability==0.7.0'],
    cmdclass=dict(build_ext=CMakeBuild),
    include_package_data=True,
    zip_safe=False,
)
