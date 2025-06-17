cmake_minimum_required(VERSION 3.20)
project(allegro_api)

set(ALLEGRO_API_VERSION 0.0.0)

add_compile_options(-std=c++17)

find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED src/allegro_api.cpp src/allegro_api_implementation.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)
target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>)

add_executable(allegro_example src/example.cpp)
target_link_libraries(allegro_example ${PROJECT_NAME})
target_include_directories(allegro_example PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_executable(allegro_cli src/cli.cpp)
target_link_libraries(allegro_cli ${PROJECT_NAME})
target_include_directories(allegro_cli PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

set(INCLUDE_INSTALL_DIR include/ CACHE PATH INTERNAL)
set(BIN_INSTALL_DIR bin/ CACHE PATH INTERNAL)
set(LIB_INSTALL_DIR lib/ CACHE PATH INTERNAL)
set(SAHRE_INSTALL_DIR share/ CACHE PATH INTERNAL)

# Install
install(TARGETS ${PROJECT_NAME} allegro_example allegro_cli
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY systemd
  DESTINATION share/${PROJECT_NAME}/
)

include(CMakePackageConfigHelpers)

configure_package_config_file(cmake/allegro_apiConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/allegro_apiConfig.cmake
  INSTALL_DESTINATION ${SAHRE_INSTALL_DIR}/cmake/allegro_api
  PATH_VARS INCLUDE_INSTALL_DIR ALLEGRO_API_VERSION)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/allegro_apiConfigVersion.cmake
  VERSION ${ALLEGRO_API_VERSION} COMPATIBILITY SameMajorVersion )

install(EXPORT ${PROJECT_NAME} DESTINATION "${SAHRE_INSTALL_DIR}/cmake/allegro_api")

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/allegro_apiConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/allegro_apiConfigVersion.cmake
        DESTINATION ${SAHRE_INSTALL_DIR}/cmake/allegro_api )