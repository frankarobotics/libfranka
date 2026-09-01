import importlib.util
import os
import re
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# Directory of the pylibfranka python project (this file's location).
ROOT_DIR = Path(__file__).parent
# Repository root: the C++ libfranka project one level up. The bindings are a
# subproject; the shared version lives in the top-level CMakeLists.txt and the
# native build is driven from the repo root (add_subdirectory(pylibfranka)).
REPO_ROOT = ROOT_DIR.parent


def load_setup_helpers():
    helper_path = ROOT_DIR / "setup_helpers.py"
    spec = importlib.util.spec_from_file_location("pylibfranka_setup_helpers", helper_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load setup helpers from {helper_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SETUP_HELPERS = load_setup_helpers()


def get_version():
    """Extract version from the top-level (repo root) CMakeLists.txt.

    Fails loudly rather than silently defaulting: a wrong version would produce
    a publishable-but-mislabelled artifact. This also surfaces the case where
    pylibfranka is built outside the libfranka source tree (e.g. from an sdist
    that does not vendor the repo root), instead of shipping ``0.0.0``.
    """
    cmake_file = REPO_ROOT / "CMakeLists.txt"
    if cmake_file.exists():
        content = cmake_file.read_text(encoding="utf-8")
        match = re.search(r"set\(libfranka_VERSION\s+(\d+\.\d+\.\d+)\)", content)
        if match:
            return match.group(1)
    raise RuntimeError(
        f"Could not determine libfranka version: expected 'set(libfranka_VERSION X.Y.Z)' "
        f"in {cmake_file}. pylibfranka must be built from within the libfranka source tree."
    )


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
            f"-DGENERATE_PYLIBFRANKA=ON",
            f"-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
            "-DCMAKE_BUILD_TYPE=Release",
        ]

        # Use pip-installed pybind11 if available (ensures Python version compatibility)
        pybind11_dir = get_pybind11_cmake_dir()
        if pybind11_dir:
            cmake_args.append(f"-Dpybind11_DIR={pybind11_dir}")

        configure = subprocess.run(
            ["cmake", ext.sourcedir] + cmake_args,
            cwd=build_temp,
            capture_output=True,
            text=True,
        )
        if configure.returncode != 0:
            sys.stdout.write(configure.stdout)
            sys.stderr.write(configure.stderr)
            guidance = SETUP_HELPERS.explain_cmake_configure_failure(
                configure.stdout, configure.stderr
            )
            if guidance:
                raise RuntimeError(guidance)
            configure.check_returncode()
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

        # Post-build step: generate .pyi type stubs so LSP servers / IDEs get
        # type hints for the compiled bindings (pybind11 does not emit stubs).
        self.generate_stubs(ext, extdir)

    def generate_stubs(self, ext, extdir):
        """Generate PEP 561 stub files (.pyi) for the compiled extension.

        pybind11-stubgen imports the freshly built module and introspects it, so
        the extension (and its runtime dependencies, e.g. libfranka.so) must be
        importable. Stubs are written directly into the build-tree package
        directory (``extdir``, i.e. ``build/lib.../pylibfranka``), alongside the
        compiled ``.so``. This is what setuptools packages into the wheel, and it
        sidesteps the fact that ``build_py`` (which copies ``package_data``) runs
        *before* ``build_ext`` -- so writing into the source tree would be too
        late to be picked up.
        """
        # ``extdir`` is ``<build_lib>/pylibfranka``; its parent is the build-lib
        # root that must be on PYTHONPATH so ``import pylibfranka._pylibfranka``
        # resolves to the *built* package (which contains the compiled .so),
        # not the source tree (which does not).
        package_dir = Path(extdir)
        build_lib_root = package_dir.parent
        stub_file = package_dir / f"{ext.name.split('.')[-1]}.pyi"
        py_typed = package_dir / "py.typed"

        # Explicit opt-out: skip generation entirely and ensure the wheel does not
        # advertise types it does not ship (drop any stub/marker copied in by
        # build_py from the source tree).
        if os.environ.get("PYLIBFRANKA_SKIP_STUBS"):
            print("WARNING: skipping .pyi stub generation (PYLIBFRANKA_SKIP_STUBS set)")
            stub_file.unlink(missing_ok=True)
            py_typed.unlink(missing_ok=True)
            return

        # Never ship a stale stub: start from a clean slate for this build.
        stub_file.unlink(missing_ok=True)

        env = dict(os.environ)
        python_path = [str(build_lib_root), env.get("PYTHONPATH", "")]
        env["PYTHONPATH"] = os.pathsep.join(p for p in python_path if p)

        try:
            subprocess.check_call(
                [
                    sys.executable,
                    "-m",
                    "pybind11_stubgen",
                    ext.name,  # pylibfranka._pylibfranka
                    "--output-dir",
                    str(build_lib_root),
                ],
                cwd=str(build_lib_root),
                env=env,
            )
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            # Stub generation is part of the deliverable (typed wheels), so a
            # failure is fatal. Set PYLIBFRANKA_SKIP_STUBS=1 to opt out.
            raise RuntimeError(
                "pybind11-stubgen failed to generate type stubs. Install "
                "'pybind11-stubgen' (a build dependency) or set "
                "PYLIBFRANKA_SKIP_STUBS=1 to build an untyped wheel."
            ) from exc

        if not stub_file.exists():
            raise RuntimeError(f"pybind11-stubgen ran but produced no stub at {stub_file}")

        # PEP 561 marker so type checkers discover the stubs in the installed wheel.
        py_typed.write_text("")


setup(
    name="pylibfranka",
    version=_version,
    packages=["pylibfranka"],
    python_requires=">=3.9",
    install_requires=["numpy>=1.19.0"],
    ext_modules=[CMakeExtension("pylibfranka._pylibfranka", sourcedir=str(REPO_ROOT))],
    cmdclass={
        "build_ext": CMakeBuild,
    },
    zip_safe=False,
    package_data={
        "pylibfranka": ["*.so", "*.pyd", "VERSION", "*.pyi", "py.typed"],
    },
)
