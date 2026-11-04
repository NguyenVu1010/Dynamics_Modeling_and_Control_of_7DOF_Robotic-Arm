# cobot_studio (feature studio-app, ADR-021): the Qt5 view over cobot_studio_core. The widgets
# live in cobot_studio_ui so the smoke test links them; the executable adds only main.cpp.
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Charts)

add_library(cobot_studio_ui STATIC main_window.cpp studio_worker.cpp)
set_target_properties(cobot_studio_ui PROPERTIES AUTOMOC ON)
cobot_set_target_defaults(cobot_studio_ui)
target_include_directories(cobot_studio_ui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cobot_studio_ui PUBLIC cobot_studio_core Qt5::Widgets Qt5::Charts)

add_executable(cobot_studio main.cpp)
set_target_properties(cobot_studio PROPERTIES AUTOMOC ON)
cobot_set_target_defaults(cobot_studio)
target_compile_definitions(cobot_studio PRIVATE COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(cobot_studio PRIVATE cobot_studio_ui cobot_studio_core Qt5::Widgets Qt5::Charts)
install(TARGETS cobot_studio RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
