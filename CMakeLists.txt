cmake_minimum_required(VERSION 3.10.0)
project(matlab_damo VERSION 0.1.0 LANGUAGES C CXX)

set(MATLAB_FILE_NAME stewart_V2_c)

include_directories(
    ${PROJECT_SOURCE_DIR}/${MATLAB_FILE_NAME}/inc
    ${PROJECT_SOURCE_DIR}/include
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

file(GLOB MATLAB_SOURES ${PROJECT_SOURCE_DIR}/${MATLAB_FILE_NAME}/src/*.c)

add_library(libstewart_V2_c SHARED ${MATLAB_SOURES})

add_executable(matlab_damo 
                            src/main.cpp
                            src/serial.cpp
                            #${MATLAB_SOURES}
)

target_link_libraries(matlab_damo 
    jsoncpp
    Boost::system 
    Threads::Threads  
    libstewart_V2_c  
)

include(CTest)
enable_testing()

