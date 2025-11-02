cmake_minimum_required(VERSION 3.8)
project(dora2filter)

find_package(ament_cmake REQUIRED)
find_package(filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(dora2_filter SHARED
  src/dora2_filter.cpp
)

target_include_directories(dora2_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dora2_filter
  filters
  pluginlib
  rclcpp
  sensor_msgs
)

ament_export_dependencies(filters pluginlib rclcpp sensor_msgs)
ament_export_include_directories(include)
ament_export_libraries(dora2_filter)

pluginlib_export_plugin_description_file(filters dora2_filter_plugins.xml)

install(
  TARGETS dora2_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES dora2_filter_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  get_target_property(_pluginlib_library pluginlib::pluginlib IMPORTED_LOCATION)
  if(NOT _pluginlib_library)
    get_target_property(_pluginlib_library pluginlib::pluginlib LOCATION)
  endif()
  set(_extra_library_dirs "")
  if(_pluginlib_library)
    get_filename_component(_pluginlib_library_dir "${_pluginlib_library}" DIRECTORY)
    list(APPEND _extra_library_dirs "${_pluginlib_library_dir}")
  endif()

  get_target_property(_class_loader_library class_loader::class_loader IMPORTED_LOCATION)
  if(NOT _class_loader_library)
    get_target_property(_class_loader_library class_loader::class_loader LOCATION)
  endif()
  if(_class_loader_library)
    get_filename_component(_class_loader_library_dir "${_class_loader_library}" DIRECTORY)
    list(APPEND _extra_library_dirs "${_class_loader_library_dir}")
  endif()

  list(REMOVE_DUPLICATES _extra_library_dirs)

  ament_add_gtest(test_dora2_filter
    test/test_dora2_filter.cpp
    APPEND_LIBRARY_DIRS
      ${_extra_library_dirs}
  )
  target_link_libraries(test_dora2_filter
    dora2_filter
  )
  ament_target_dependencies(test_dora2_filter
    rclcpp
    sensor_msgs
    pluginlib
  )
endif()

ament_package()
