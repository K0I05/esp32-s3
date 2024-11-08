# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

# embed_txtfiles tag is required in the platformio.ini and cmakelist.txt files
# compiled on Visual Studio Code with ESP-IDF v5.3.1 and PIO 6.1.16 Core
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)
idf_component_register(SRCS ${app_sources}
                    INCLUDE_DIRS ".")