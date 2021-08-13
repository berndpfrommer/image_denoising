cmake_minimum_required(VERSION 3.0.2)
project(image_denoising)

add_compile_options(-Wall -Wextra -pedantic -Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  image_transport
  cv_bridge
  )

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/NLMeansDyn.cfg
  cfg/BilateralDyn.cfg
  )

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

#
# ------- filter library has all the filters
#
add_library(filter src/filter.cpp src/nl_means_filter.cpp src/bilateral_filter.cpp)
target_link_libraries(filter ${OpenCV_LIBS})

#
# ------ denoiser does pub/subscribe etc
#
add_library(denoiser src/denoiser.cpp)
target_link_libraries(denoiser filter ${OpenCV_LIBS} ${catkin_LIBRARIES})

# denoiser nodelet
add_library(denoiser_nodelet src/denoiser_nodelet.cpp)
target_link_libraries(denoiser_nodelet denoiser ${catkin_LIBRARIES})

# denoiser node
add_executable(denoiser_node src/denoiser_node.cpp)
target_link_libraries(denoiser_node denoiser ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS denoiser_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS denoiser filter denoiser_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
 
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  )


#############
## Testing ##
#############

# To be done...
