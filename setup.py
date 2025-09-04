import os
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        build_temp = os.path.join(self.build_temp, "build")
        if not os.path.exists(build_temp):
            os.makedirs(build_temp)

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DBUILD_TESTS=OFF",
            f"-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
            "-DCMAKE_BUILD_TYPE=Release",
        ]

        subprocess.check_call(["cmake", ext.sourcedir] + cmake_args, cwd=build_temp)
        subprocess.check_call(
            ["cmake", "--build", ".", "--config", "Release", "--parallel"],
            cwd=build_temp,
        )

        # Ensure the built extension gets properly tracked by setuptools
        built_lib = Path(extdir) / f"{ext.name.split('.')[-1]}.so"
        if built_lib.exists():
            # Copy to the package directory to ensure it's tracked
            package_dir = Path("pylibfranka")
            if package_dir.exists():
                import shutil

                target = package_dir / built_lib.name
                if not target.exists():
                    shutil.copy2(built_lib, target)


setup(
    name="pylibfranka",
    version="0.1.0",
    packages=["pylibfranka"],
    python_requires=">=3.8",
    install_requires=["numpy>=1.19.0"],
    ext_modules=[CMakeExtension("pylibfranka._pylibfranka")],
    cmdclass={
        "build_ext": CMakeBuild,
    },
    zip_safe=False,
    package_data={
        "pylibfranka": ["*.so", "*.pyd"],
    },
)
