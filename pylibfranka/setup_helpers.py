from __future__ import annotations

import re


def explain_cmake_configure_failure(stdout: str, stderr: str) -> str | None:
    """Return an actionable packaging error for known CMake dependency failures."""
    output = "\n".join(part for part in (stdout, stderr) if part)

    match = re.search(
        r'Could not find a package configuration file provided by\s+"([^"]+)"',
        output,
        re.DOTALL,
    )
    if not match:
        match = re.search(
            r'By not providing "Find([A-Za-z0-9_+-]+)\.cmake" in CMAKE_MODULE_PATH'
            r".*?asked CMake to find a package configuration file",
            output,
            re.DOTALL,
        )
    if match:
        package_name = match.group(1)
        return (
            "Could not configure pylibfranka because CMake could not find the required native "
            f"dependency '{package_name}'. Before running 'pip install ./pylibfranka', install "
            "the full libfranka source-build dependency set (including console_bridge, "
            "TinyXML2, and Pinocchio) or point CMake to an existing installation with "
            f"{package_name}_DIR or CMAKE_PREFIX_PATH. The Python wheel build cannot install "
            "native CMake dependencies for you."
        )

    return None
