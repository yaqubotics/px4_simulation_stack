function(build_urdf input_file output_file)

    # get filename
    get_filename_component(file_name ${input_file} NAME)

    # get filename without extension
    string(REGEX MATCH ^[^.]+ model_name ${file_name})

    # add target to build urdf from xacro
    add_custom_target(model_${model_name} ALL
                      rosrun xacro xacro --inorder -o ${output_file} ${input_file})

endfunction()
