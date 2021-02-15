function(DLL_Copy_Command CMD_NAME DLL_LIST)

    # Return now if not windows
    if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      return()
    endif()

    # Remove duplicates in the input list
    list(REMOVE_DUPLICATES DLL_LIST)

    message(STATUS "==== Add custom commands for copying DLLs ====")
    message(STATUS "DLL list...")

    # Create custom target for copying DLLs; add it to the default build target
    add_custom_target(${CMD_NAME} ALL)

    # Add custom command to create the destination folder
    add_custom_command(TARGET ${CMD_NAME} PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "${EXECUTABLE_OUTPUT_PATH}/$<CONFIGURATION>/")

    # Loop over the list of all DLLs and create a custom command
    foreach(DLL ${DLL_LIST})
        message(STATUS "...${DLL}")

        # Create custom commands, invoked pre-build to copy DLLs to the appropriate
        # directory (depending on the configuration selected at build time in VS)
        add_custom_command(
            TARGET ${CMD_NAME} PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${DLL}"
                "${EXECUTABLE_OUTPUT_PATH}/$<CONFIGURATION>/"
        )
    endforeach()

    message("Attention: Only RELEASE DLLs are copied automatically.")
    message("           You must manually copy the DLLs for other configurations.")
endfunction()
