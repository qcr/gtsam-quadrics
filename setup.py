import os
from setuptools import find_packages, setup, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):

    def __init__(self, name, cmake_path):
        self.cmake_path = cmake_path
        super().__init__(name, sources=[])


class cmake_build_ext(build_ext):

    def run(self):
        for e in self.extensions:
            self.build_cmake(e)
        super().run()

    def build_cmake(self, ext):
        print("BUILD DETAILS:")
        source_dir = os.getcwd()
        build_dir = self.build_temp

        print(self.get_ext_fullpath(ext.name))

        self.spawn(['cmake', '-G', 'Ninja', '-B', build_dir, 'S', source_dir])
        self.spawn(['cmake', '--build', build_dir])


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
      ext_modules=[CMakeExtension('gtsam_quadrics', './CMakeLists.txt')],
      cmdclass={'build_ext': cmake_build_ext},
      classifiers=(
          "Development Status :: 4 - Beta",
          "Programming Language :: Python :: 3",
          "License :: OSI Approved :: BSD License",
          "Operating System :: OS Independent",
      ))
