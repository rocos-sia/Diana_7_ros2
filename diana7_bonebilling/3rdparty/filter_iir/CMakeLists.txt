cmake_minimum_required (VERSION 3.5)

project(filter_iir)

FILE(GLOB FILES_SOURCE src/*.cpp)

add_library(filter_iir STATIC   ${FILES_SOURCE} )

target_include_directories(filter_iir   
  PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/iir_filters
  )
  
# add_library(SUN::filter_iir ALIAS filter_iir)