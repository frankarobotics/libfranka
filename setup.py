import os
import re
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


def get_version():
    """Extract version from CMakeLists.txt."""
    cmake_file = Path(__file__).parent / "CMakeLists.txt"
    if cmake_file.exists():
        with open(cmake_file, 'r') as f:
            content = f.read()
            match = re.search(r'set\(libfranka_VERSION\s+(\d+\.\d+\.\d+)\)', content)
            if match:
                return match.group(1)
    return "0.0.0"


def update_version_file():
    """Update _version.py with current version from CMakeLists.txt."""
    version = get_version()
    version_file = Path(__file__).parent / "pylibfranka" / "_version.py"
    
    if version_file.exists():
        # Read current content
        with open(version_file, 'r') as f:
            content = f.read()
        
        # Update the version line (matches the pattern with AUTO-GENERATED comment)
        new_content = re.sub(
            r'__version__ = "[^"]*"  # AUTO-GENERATED',
            f'__version__ = "{version}"  # AUTO-GENERATED',
            content
        )
        
        # Write the updated content
        with open(version_file, 'w') as f:
            f.write(new_content)
    else:
        # Create the file if it doesn't exist
        with open(version_file, 'w') as f:
            f.write(f'''"""Version information for pylibfranka."""

__all__ = ['__version__']


# Version will be written here by pip install . command
# or setup.py during build/install
# DO NOT EDIT THIS LINE - it is automatically replaced
__version__ = "{version}"  # AUTO-GENERATED
''')


# Update version file immediately when setup.py is loaded
update_version_file()


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        # Ensure version file is updated (redundant but safe)
        update_version_file()
        
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
    version=get_version(),
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
