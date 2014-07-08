cmake_minimum_required(VERSION 2.8)

project(Lightstage_geometry)

set(EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})


find_package( OpenCV REQUIRED )


include_directories("/home/alou/trimesh2/include"
                    "/home/alou/rapidxml-1.13/")

link_directories("/home/alou/trimesh2/lib.Linux64"
        #"/home/alou/SuiteSparse/AMD/Lib"
        #"/home/alou/SuiteSparse/CAMD/Lib"
        #"/home/alou/SuiteSparse/CHOLMOD/Lib/"
        #"/home/alou/SuiteSparse/SuiteSparse_config/"
        #"/home/alou/SuiteSparse/COLAMD/Lib"
        #"/home/alou/SuiteSparse/"
)


add_executable(Lightstage_geometry main.cpp)

target_link_libraries( Lightstage_geometry ${OpenCV_LIBS} trimesh gomp)


