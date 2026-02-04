# Try to find gtest package. If available, take it. Otherwise, try to download.
find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
  message(STATUS "googletest not found. Fetching googletest...")
  include(FetchContent)

  FetchContent_Declare(
    gtest
    GIT_REPOSITORY https://github.com/google/googletest
    GIT_TAG        v1.15.2)
    
  FetchContent_MakeAvailable(gtest)

  set_target_properties(gtest PROPERTIES POSITION_INDEPENDENT_CODE ON)

  message(STATUS "Fetched googletest: 1.15.2")
else()
  message(STATUS "Found googletest: ${GTest_VERSION}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
