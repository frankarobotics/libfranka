from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from setup_helpers import explain_cmake_configure_failure


def test_explain_cmake_configure_failure_for_missing_pinocchio():
    message = explain_cmake_configure_failure(
        "",
        """
        CMake Error at CMakeLists.txt:54 (find_package):
          By not providing "Findpinocchio.cmake" in CMAKE_MODULE_PATH this project
          has asked CMake to find a package configuration file provided by
          "pinocchio", but CMake did not find one.
        """,
    )

    assert message is not None
    assert "pinocchio" in message
    assert "pip install ./pylibfranka" in message
    assert "pinocchio_DIR" in message
    assert "CMAKE_PREFIX_PATH" in message
    assert "console_bridge" in message


def test_explain_cmake_configure_failure_for_missing_console_bridge():
    message = explain_cmake_configure_failure(
        "",
        """
        CMake Error at CMakeLists.txt:51 (find_package):
          Could not find a package configuration file provided by "console_bridge"
          with any of the following names:

            console_bridgeConfig.cmake
            console_bridge-config.cmake
        """,
    )

    assert message is not None
    assert "console_bridge" in message
    assert "console_bridge_DIR" in message


def test_explain_cmake_configure_failure_ignores_other_errors():
    assert explain_cmake_configure_failure("", "CMake Error: unrelated failure") is None


def test_explain_cmake_configure_failure_ignores_incidental_find_cmake_mentions():
    assert (
        explain_cmake_configure_failure(
            "",
            'See docs for Findpinocchio.cmake lookup order, but this is not a missing package error.',
        )
        is None
    )
