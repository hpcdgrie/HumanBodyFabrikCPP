set(HEADERS
constraints.h
fabrik.h
)
set(SOURCES
constraints.cpp
fabrik.cpp
)
include(GNUInstallDirs)

find_package(OpenSceneGraph COMPONENTS REQUIRED)
add_library(fabrik ${HEADERS} ${SOURCES})
target_link_libraries(fabrik PUBLIC ${OPENSCENEGRAPH_LIBRARIES})
target_include_directories(fabrik PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS} "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>" "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
#add_subdirectory(angle_calculator)

# make cache variables for install destinations

install(TARGETS fabrik
        EXPORT fabrikTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT fabrikTargets
        FILE fabrikTargets.cmake
        NAMESPACE fabrik::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fabrik
)
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/fabrikConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/fabrikConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fabrik
)

install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/fabrikConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/fabrikConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fabrik
)
export(
    EXPORT fabrikTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/fabrikTargets.cmake"
    NAMESPACE fabrik::)