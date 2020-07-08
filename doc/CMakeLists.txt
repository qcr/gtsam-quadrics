	# gtsam_quadrics core subfolders
	set(QUADRICSLAM_doc_subdirs 
		gtsam_quadrics/base
		gtsam_quadrics/geometry
	)

	# set(doc_subdirs ${QUADRICSLAM_doc_subdirs})

	# From subfolders, build a list with whitespace separation of paths
	set(QUADRICSLAM_DOXYGEN_INPUT_PATHS "")
	foreach(dir ${QUADRICSLAM_doc_subdirs})
		set(QUADRICSLAM_DOXYGEN_INPUT_PATHS "${QUADRICSLAM_DOXYGEN_INPUT_PATHS} ${PROJECT_SOURCE_DIR}/${dir}")
	endforeach()

	# Generate Doxyfile
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

	# Add target to actually build documentation as configured
	add_custom_target(doc
			${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
			COMMENT "Generating API documentation with Doxygen" VERBATIM
	)

	# Clean target
	add_custom_target(doc_clean
			COMMAND
					cmake -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/latex
			COMMAND
					cmake -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/html
			COMMENT "Removing Doxygen documentation"
	)