from setuptools import find_packages, setup, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):

    def __init__(self):
        super().__init__('', sources=[])


class cmake_build_ext(build_ext):

    def run(self):
        super().run()


with open("README.md", 'r') as f:
    long_description = f.read()

setup(name='gtsam_quadrics',
      version='0.0.1',
      author='Lachlan Nicholson',
      author_email='TODO@qut.edu.au',
      url='https://github.com/best-of-acrv/gtsam_quadrics',
      description='Quadrics for GTSAM',
      long_description=long_description,
      long_description_content_type='text/markdown',
      packages=find_packages(),
      install_requires=[],
      ext_modules=[CMakeExtension('g2o_cpp', './g2o/cpp/CMakeLists.txt')],
      cmdclass={'build_ext': cmake_build_ext},
      classifiers=(
          "Development Status :: 4 - Beta",
          "Programming Language :: Python :: 3",
          "License :: OSI Approved :: BSD License",
          "Operating System :: OS Independent",
      ))
