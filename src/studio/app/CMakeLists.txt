# cobot_studio (feature studio-app, ADR-021): the Qt5 view over cobot_studio_core. The widgets
# live in cobot_studio_ui so the smoke test links them; the executable adds only main.cpp.
# The 3D view (feature studio-3d) uses Qt3D: Qt3DWindow in a widget container. The Control tab
# (feature studio-control) drives a controller through cobot_studio_core's BridgeClient.
# The theme (studio config UI design 3.5): the vendored Breeze light sheet and studio.qss, compiled
# in as Qt resources (AUTORCC); theme.cpp registers them with Q_INIT_RESOURCE.
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Charts 3DCore 3DRender 3DExtras)

add_library(cobot_studio_ui STATIC main_window.cpp param_card.cpp param_spin.cpp studio_worker.cpp
  robot_view.cpp
  control_tab.cpp verify_tab.cpp time_cursor.hpp theme.cpp studio.qrc
  ${PROJECT_SOURCE_DIR}/third_party/breeze/breeze.qrc)
set_target_properties(cobot_studio_ui PROPERTIES AUTOMOC ON AUTORCC ON)
cobot_set_target_defaults(cobot_studio_ui)
target_include_directories(cobot_studio_ui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cobot_studio_ui PUBLIC cobot_studio_core Qt5::Widgets Qt5::Charts Qt5::3DCore
  Qt5::3DRender Qt5::3DExtras)

add_executable(cobot_studio main.cpp)
set_target_properties(cobot_studio PROPERTIES AUTOMOC ON)
cobot_set_target_defaults(cobot_studio)
target_compile_definitions(cobot_studio PRIVATE COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(cobot_studio PRIVATE cobot_studio_ui cobot_studio_core Qt5::Widgets Qt5::Charts)
install(TARGETS cobot_studio RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
