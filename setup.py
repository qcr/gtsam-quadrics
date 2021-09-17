import os
from setuptools import find_packages, setup, Extension
from setuptools.command.build_ext import build_ext
import shutil


class CMakeExtension(Extension):

    def __init__(self, name, cmake_path):
        self.cmake_path = cmake_path
        super().__init__(name, sources=[])


class cmake_build_ext(build_ext):

    def run(self):
        self.build_cmake(self.extensions)
        super().run()

    def build_cmake(self, exts):
        # This code assumes 'gtsam' & 'gtsam_quadrics' extensions are specified
        # in that order...
        gtsam_ext = exts[0]
        gtsam_quadrics_ext = exts[1]

        # Build our CPython shared objects
        source_dir = os.getcwd()
        build_dir = self.build_temp

        self.spawn([
            'cmake', '-DBUILD_SHARED_LIBS=OFF',
            '-DCMAKE_POSITION_INDEPENDENT_CODE=ON', '-G', 'Ninja', '-B',
            build_dir, '-S', source_dir
        ])
        self.spawn(['cmake', '--build', build_dir])

        # Move shared objects to the expected output location
        lib_dir = os.path.dirname(self.get_ext_fullpath(gtsam_ext.name))
        os.makedirs(lib_dir, exist_ok=True)

        shutil.copy(
            os.path.join(build_dir, 'gtsam', 'python', 'gtsam',
                         self.get_ext_filename(gtsam_ext.name)),
            self.get_ext_fullpath(gtsam_ext.name))
        shutil.copy(
            os.path.join(build_dir,
                         self.get_ext_filename(gtsam_quadrics_ext.name)),
            self.get_ext_fullpath(gtsam_quadrics_ext.name))


with open("README.md", 'r') as f:
    long_description = f.read()

setup(name='gtsam_quadrics',
      version='0.1.0',
      author='Lachlan Nicholson',
      author_email='lachlan.nicholson@hdr.qut.edu.au',
      maintainer='Ben Talbot',
      maintainer_email='b.talbot@qut.edu.au',
      url='https://github.com/best-of-acrv/gtsam_quadrics',
      description='Quadrics for GTSAM',
      long_description=long_description,
      long_description_content_type='text/markdown',
      packages=find_packages(),
      install_requires=[],
      ext_modules=[
          CMakeExtension('gtsam', './gtsam/CMakeLists.txt'),
          CMakeExtension('gtsam_quadrics', './CMakeLists.txt')
      ],
      cmdclass={'build_ext': cmake_build_ext},
      classifiers=(
          "Development Status :: 4 - Beta",
          "Programming Language :: Python :: 3",
          "License :: OSI Approved :: BSD License",
          "Operating System :: OS Independent",
      ))
