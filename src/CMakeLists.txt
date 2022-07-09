cmake_minimum_required (VERSION 3.10)
PROJECT(evaluate_model)

FILE(GLOB SOURCE_FILE *.cpp *.h)

add_library(${PROJECT_NAME} ${SOURCE_FILE})

link_private(${PROJECT_NAME})