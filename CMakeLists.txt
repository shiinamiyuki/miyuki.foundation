cmake_minimum_required(VERSION 3.1)

project(miyuki.foundation)
set(CMAKE_CXX_STANDARD  17)
add_subdirectory(external/fmt)

file(GLOB libSRC src/*.* src/*/*.* include/*/*.*)
add_library(foundation ${libSRC})
target_include_directories(foundation PUBLIC include/)
target_include_directories(foundation PUBLIC external/cereal/include)
target_include_directories(foundation PUBLIC external/glm)
target_include_directories(foundation PUBLIC external/json/include)
target_include_directories(foundation PUBLIC external/stb)
target_include_directories(foundation PUBLIC external/fmt/include)