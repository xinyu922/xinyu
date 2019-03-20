cmake_minimum_required(VERSION 2.8)
set(TARGET_NAME Leadsense_Capture)

if(DEFINED PROJECT_NAME)
	set(insideSDK true)
endif(DEFINED PROJECT_NAME)

if(insideSDK)
	# inside sdk project
	SET(EvoBinoSDK_LIBRARIES evo_depthcamera)
else()
	# external build
	PROJECT( ${TARGET_NAME} )
	
	set(CMAKE_CXX_STANDARD 11) #C++ 11...
	set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
	set(CMAKE_CXX_EXTENSIONS OFF) #..without compiler extensions like gnu++11

	FIND_PACKAGE(EvoBinoSDK REQUIRED)
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}     "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
	
endif()

#find_package(PCL REQUIRED)
	
if (EvoBinoSDK_WITH_CUDA)

	find_package(OpenCV REQUIRED)
	find_package(Eigen3 REQUIRED)
	find_package(Threads)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
        include_directories(${EvoBinoSDK_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

	if(insideSDK)

		ADD_EXE_MODULE(${TARGET_NAME} ${FOLDERNAME})
		set_target_properties (${TARGET_NAME} PROPERTIES FOLDER ${FOLDERNAME})
	else()
		file(GLOB DIR_HEADERS "*.h")
		file(GLOB DIR_CPP_SRCS "capture_xinyu.cpp")
		add_executable(${TARGET_NAME} ${DIR_HEADERS} ${DIR_CPP_SRCS})

	endif()

        target_link_libraries(${TARGET_NAME} ${EvoBinoSDK_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

endif (EvoBinoSDK_WITH_CUDA)
