# CMake configuration for test targets

if(BUILD_GS)
  add_executable(${PROJECT_NAME}_test_hash test_hash.cpp)
  target_link_libraries(${PROJECT_NAME}_test_hash ${PROJECT_NAME}.gp3d pthread)
  add_test(NAME ${PROJECT_NAME}_test_hash COMMAND ${PROJECT_NAME}_test_hash)

  find_package(Open3D QUIET)
  if(Open3D_FOUND)
    add_library(${PROJECT_NAME}.gmm gmm/GMMFit.cpp)
    target_include_directories(${PROJECT_NAME}.gmm PRIVATE gmm ${Open3D_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME}.gmm ${gs_libs} Open3D::Open3D)

    add_executable(${PROJECT_NAME}_test_gmm test_gmm.cpp)
    target_include_directories(${PROJECT_NAME}_test_gmm PRIVATE gmm)
    target_link_libraries(${PROJECT_NAME}_test_gmm ${PROJECT_NAME}.gmm)
    add_test(NAME ${PROJECT_NAME}_test_gmm COMMAND ${PROJECT_NAME}_test_gmm)

    add_executable(${PROJECT_NAME}_test_ecli test_ecli.cpp)
    target_include_directories(${PROJECT_NAME}_test_ecli PRIVATE ${Open3D_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME}_test_ecli ${gs_libs} Open3D::Open3D)
    add_test(NAME ${PROJECT_NAME}_test_ecli COMMAND ${PROJECT_NAME}_test_ecli)
  endif()
endif()
