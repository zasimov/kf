set(CMAKE_LEGACY_CYGWIN_WIN32 0)
cmake_minimum_required (VERSION 2.8.4)
project (KF)

set(EIGEN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/eigen-git-mirror)
include_directories(${EIGEN_PATH})
set(UWS_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/uWebSockets)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   # Mac OS X specific code
   include_directories(/usr/local/opt/openssl/include)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(OpenSSL REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_library(UV_LIBRARY NAMES uv)
find_library(UWS_LIBRARY NAMES uWS PATHS ${UWS_LIB_PATH} NO_DEFAULT_PATH)
find_library(M_LIBRARY NAMES m)

add_executable(kf main.cpp gaussian.cpp math.cpp parser.cpp protocol.cpp logging.cpp ekf.cpp ukf.cpp fusion.cpp fusionekf.cpp fusionukf.cpp ctrv.cpp)
set_property(TARGET kf PROPERTY CXX_STANDARD 11)
target_link_libraries(kf ${UV_LIBRARY} ${UWS_LIBRARY} ${ZLIB_LIBRARIES} ${OPENSSL_LIBRARIES})

add_executable(objpose objpose.cpp gaussian.cpp math.cpp parser.cpp logging.cpp ekf.cpp ctrv.cpp ukf.cpp fusion.cpp fusionekf.cpp fusionukf.cpp)
set_property(TARGET objpose PROPERTY CXX_STANDARD 11)

find_package(GTest)
IF (GTEST_FOUND)
   enable_testing()
   include_directories(${GTEST_INCLUDE_DIRS})

   add_executable(testparser parser.cpp parser_test.cpp)
   # FIXME: -lpthread and -lm is required for Linux only
   target_link_libraries(testparser ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${M_LIBRARY})

   add_executable(testmath math.cpp logging.cpp math_test.cpp)
   target_link_libraries(testmath ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${M_LIBRARY})

   add_executable(testgaussian gaussian.cpp math.cpp logging.cpp gaussian_test.cpp)
   target_link_libraries(testgaussian ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${M_LIBRARY})

   add_executable(testctrv ctrv.cpp math.cpp logging.cpp gaussian.cpp ctrv_test.cpp)
   target_link_libraries(testctrv ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${M_LIBRARY})

   add_executable(testmp measurement_package_test.cpp)
   target_link_libraries(testmp ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${M_LIBRARY})

   add_test(NAME parser COMMAND testparser)
   add_test(NAME math COMMAND testmath)
   add_test(NAME gaussian COMMAND testgaussian)
   add_test(NAME ctrv COMMAND testctrv)
   add_test(NAME mp COMMAND testmp)
ENDIF(GTEST_FOUND)

add_executable(testfile parser.cpp testfile.cpp)
