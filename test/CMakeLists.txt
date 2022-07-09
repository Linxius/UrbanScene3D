cmake_minimum_required (VERSION 3.17)
PROJECT(evaluate_test)

FILE(GLOB SOURCE_FILE *.cpp *.h)

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_include_directories(${PROJECT_NAME} PUBLIC ../src)
target_link_libraries(${PROJECT_NAME} PRIVATE evaluate_model)
link_private(${PROJECT_NAME})
