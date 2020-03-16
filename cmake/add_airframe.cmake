function(add_airframe pack_airframe_path)

    # Airframe file paths
    if($ENV{ROS_HOME})
    set(local_airframe_path "$ENV{ROS_HOME}/etc")
    else()
    set(local_airframe_path "$ENV{HOME}/.ros/etc")
    endif()

    # aborts if directory where airframe config is put does not exist
    if(NOT EXISTS ${local_airframe_path})
        file(MAKE_DIRECTORY ${local_airframe_path}/init.d-posix)
    endif()

    # get filename
    get_filename_component(file_name ${pack_airframe_path} NAME)
    # replace airframe number by "airframe_" to obtain target name
    string(REGEX REPLACE ^[0-9]+_ "airframe_" target_name ${file_name})

    # add target that makes symbolic link
    add_custom_target(${target_name} ALL
                      ln -s -b ${pack_airframe_path} ${local_airframe_path}/init.d-posix/${file_name})

endfunction()