cmake_minimum_required(VERSION 3.1)

project(miyuki.foundation)
set(CMAKE_CXX_STANDARD  17)
add_subdirectory(external/cereal)

include_directories(external/cereal/include)
include_directories(external/glm)
include_directories(external/json/include)
include_directories(external/stb)

include_directories(include/)
file(GLOB libSRC src/*.* src/*/*.* include/*/*.*)
add_library(foundation ${libSRC})