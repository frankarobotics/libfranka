# Try to find gtest package. If available, take it. Otherwise, try to download.
find_package(googletest QUIET)
if(NOT googletest_FOUND)
  message(STATUS "googletest not found. Fetching googletest...")
  include(FetchContent)

  FetchContent_Declare(
    gtest
    GIT_REPOSITORY https://github.com/google/googletest
    GIT_TAG        v1.15.2)
  FetchContent_GetProperties(gtest)
  if(NOT gtest_POPULATED)
      FetchContent_Populate(gtest)
      add_subdirectory(${gtest_SOURCE_DIR} ${gtest_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()

  set_target_properties(gtest PROPERTIES POSITION_INDEPENDENT_CODE ON)

  if(NOT gtest_POPULATED)
    message(FATAL_ERROR "Failed to fetch googletest. Please install googletest or visit https://github.com/google/googletest")
  endif()
else()
  message(STATUS "Found googletest: ${gtest_VERSION}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
