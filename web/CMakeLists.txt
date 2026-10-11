# In-browser build: the full engagement scenario compiled to WebAssembly plus a
# raylib/WebGL2 viewer, packaged as a static page (index.html/.js/.wasm/.data)
# that any web server can host. Configure with the wasm-release preset.

include(FetchContent)

# raylib for the browser on WebGL2 (GLES 3.0): the city renderer needs native
# instancing, vertex array objects, and per-instance attribute divisors.
set(PLATFORM "Web" CACHE STRING "" FORCE)
set(OPENGL_VERSION "ES 3.0" CACHE STRING "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)
FetchContent_MakeAvailable(raylib)

# Bake the compact, tiled, deflated map (4.5 MB vs the 13 MB desktop asset).
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(WEB_ASSET_DIR ${CMAKE_CURRENT_BINARY_DIR}/web_assets)
set(WEB_MAP ${WEB_ASSET_DIR}/un_map.webbin)
add_custom_command(
    OUTPUT ${WEB_MAP}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${WEB_ASSET_DIR}
    COMMAND Python3::Interpreter ${PROJECT_SOURCE_DIR}/tools/gen_web_map.py
            ${PROJECT_SOURCE_DIR}/assets/un_map.bin ${WEB_MAP}
    DEPENDS ${PROJECT_SOURCE_DIR}/tools/gen_web_map.py
            ${PROJECT_SOURCE_DIR}/assets/un_map.bin
    COMMENT "Baking the compact web map"
    VERBATIM)
add_custom_target(web_map DEPENDS ${WEB_MAP})

add_executable(web_viewer web_viewer.cpp ${PROJECT_SOURCE_DIR}/src/SimConfig.cpp)
target_link_libraries(web_viewer PRIVATE sim_core ${SIM_YAMLCPP} raylib)
# raymath.h trips -Wmissing-field-initializers on every inline helper.
target_compile_options(web_viewer PRIVATE -Wall -Wextra -Wno-missing-field-initializers)
add_dependencies(web_viewer web_map)
set_target_properties(web_viewer PROPERTIES OUTPUT_NAME "index" SUFFIX ".html")

set(WEB_SHELL ${CMAKE_CURRENT_SOURCE_DIR}/shell.html)
set_property(TARGET web_viewer APPEND PROPERTY LINK_DEPENDS ${WEB_SHELL} ${WEB_MAP})
target_link_options(web_viewer PRIVATE
    "SHELL:--shell-file \"${WEB_SHELL}\""
    "SHELL:-s USE_GLFW=3"
    "SHELL:-s MIN_WEBGL_VERSION=2"
    "SHELL:-s MAX_WEBGL_VERSION=2"
    "SHELL:-s ALLOW_MEMORY_GROWTH=1"
    "SHELL:-s INITIAL_MEMORY=128MB"
    "SHELL:-s STACK_SIZE=1MB"
    "SHELL:-s EXPORTED_RUNTIME_METHODS=[\"ccall\"]"
    "SHELL:-s EXPORTED_FUNCTIONS=[\"_main\",\"_web_start\",\"_web_set_paused\",\"_web_set_speed\",\"_web_set_lift\",\"_web_resize\",\"_web_orbit\",\"_web_pan\",\"_web_zoom\",\"_web_tap\",\"_web_free_camera\",\"_web_hover\",\"_web_reset_view\",\"_web_bench\",\"_web_bench_sim\"]"
    "SHELL:--preload-file \"${WEB_MAP}@/assets/un_map.webbin\""
    "SHELL:--preload-file \"${PROJECT_SOURCE_DIR}/config/simulation.yaml@/config/simulation.yaml\""
    "SHELL:--preload-file \"${PROJECT_SOURCE_DIR}/assets/Roboto-Regular.ttf@/assets/Roboto-Regular.ttf\""
)
