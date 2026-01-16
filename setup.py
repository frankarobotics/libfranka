import os
import re
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# Root directory of the project
ROOT_DIR = Path(__file__).parent


def get_version():
    """Extract version from CMakeLists.txt."""
    cmake_file = ROOT_DIR / "CMakeLists.txt"
    if cmake_file.exists():
        with open(cmake_file, "r", encoding="utf-8") as f:
            content = f.read()
            match = re.search(r"set\(libfranka_VERSION\s+(\d+\.\d+\.\d+)\)", content)
            if match:
                return match.group(1)
    return "0.0.0"


def write_version_files(version):
    """Write version to VERSION file and _version.py for runtime access."""
    pylibfranka_dir = ROOT_DIR / "pylibfranka"

    # Write VERSION file (used by pyproject.toml for build metadata)
    version_file = pylibfranka_dir / "VERSION"
    version_file.write_text(f"{version}\n")

    # Write _version.py (used at runtime for pylibfranka.__version__)
    version_py = pylibfranka_dir / "_version.py"
    version_py.write_text(
        '"""Version information for pylibfranka."""\n\n'
        "__all__ = ['__version__']\n\n"
        "# Version is auto-generated from CMakeLists.txt during build\n"
        f'__version__ = "{version}"\n'
    )


# Extract and write version files before setuptools processes pyproject.toml
_version = get_version()
write_version_files(_version)


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


def get_pybind11_cmake_dir():
    """Get pybind11 CMake directory from pip-installed pybind11."""
    try:
        import pybind11

        return pybind11.get_cmake_dir()
    except (ImportError, AttributeError):
        # Fallback to system pybind11 or let CMake find it
        return None


class CMakeBuild(build_ext):
    def run(self):
        # Call parent run
        super().run()

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
            f"-DGENERATE_PYLIBFRANKA=ON",
            f"-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
            "-DCMAKE_BUILD_TYPE=Release",
        ]

        # Use pip-installed pybind11 if available (ensures Python version compatibility)
        pybind11_dir = get_pybind11_cmake_dir()
        if pybind11_dir:
            cmake_args.append(f"-Dpybind11_DIR={pybind11_dir}")

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
    version=_version,
    packages=["pylibfranka"],
    python_requires=">=3.9",
    install_requires=["numpy>=1.19.0"],
    ext_modules=[CMakeExtension("pylibfranka._pylibfranka")],
    cmdclass={
        "build_ext": CMakeBuild,
    },
    zip_safe=False,
    package_data={
        "pylibfranka": ["*.so", "*.pyd", "VERSION"],
    },
)
