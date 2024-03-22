SET(SRC_LIST DataReading.cpp PlaneExtraction.cpp PlaneExtraction_RANSAC.cpp PlaneMatching.cpp PlaneMatching_Interptree.cpp PlaneFilter.cpp CameraPoseEstimation.cpp CameraPoseEstimation_ShadowSM.cpp CameraPoseEstimation_PlaneParas.cpp MapIncremental.cpp)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/ann_1.1.2/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${PROJECT_SOURCE_DIR}/ann_1.1.2/lib)

ADD_LIBRARY(sutungpo SHARED ${SRC_LIST})
target_link_libraries(sutungpo ${PCL_LIBRARIES} ${OpenCV_LIBS} ANN)
