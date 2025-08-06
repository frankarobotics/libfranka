# Use set_version_from_git to get the current version and tag from git.
# If the current commit is tagged the git tag is returned as version.
# If not, the latest tag is returned. If no tag exists in the commits history, 0.1.0 is returned.
# If a tag has never been set, it will return 0.1.0 as VERSION and the short git commit hash as TAG.
# ${MAJOR}.${MINOR}.${PATCH}
#
# USAGE: set_version_from_git(<output_version> <output_tag>)
#
# ARGUMENTS: output_version variable that will contain the version
#            output_tag variable that will contain the tag
#
# EXAMPLE:
# set_version_from_git(PROJECT_VERSION PROJECT_TAG)
# project(<project_name> VERSION ${PROJECT_VERSION})
#
function(set_version_from_git VERSION TAG)
    find_program(GIT_EXEC git REQUIRED)
    if(NOT GIT_EXEC)
        message(FATAL_ERROR "cannot execute set_version_from_git: git not found.\napt install git")
    endif()


    set(ENV{GIT_DIR} ${CMAKE_SOURCE_DIR}/.git)
    execute_process(COMMAND ${GIT_EXEC} describe --tags
                    OUTPUT_VARIABLE LOCAL_SCOPE_TAG
                    RESULT_VARIABLE GIT_NOT_FOUND
                    ERROR_QUIET)
    if(GIT_NOT_FOUND)
         message(FATAL_ERROR "cannot find a tag in git")
    endif()

    # separate the version from the commit short hash
    if(LOCAL_SCOPE_TAG)
        string(STRIP ${LOCAL_SCOPE_TAG} LOCAL_SCOPE_TAG)
        string(FIND ${LOCAL_SCOPE_TAG} "-" FIND_INDEX)
        string(SUBSTRING ${LOCAL_SCOPE_TAG} 0 ${FIND_INDEX} LOCAL_SCOPE_VERSION)
    else()
        set(LOCAL_SCOPE_VERSION "0.1.0")
        execute_process(COMMAND ${GIT_EXEC} describe --tags --always
            OUTPUT_VARIABLE LOCAL_SCOPE_TAG
            ERROR_QUIET)
        string(STRIP ${LOCAL_SCOPE_TAG} LOCAL_SCOPE_TAG)
    endif()

    set(${VERSION} ${LOCAL_SCOPE_VERSION} PARENT_SCOPE)
    set(${TAG} ${LOCAL_SCOPE_TAG} PARENT_SCOPE)
endfunction()
