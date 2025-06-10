cmake_minimum_required(VERSION 3.8)
project(spline_lib)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)

# スプラインライブラリのソースファイル
set(SPLINE_SOURCES
    spline.cpp
)

# スプラインライブラリの作成
add_library(spline_lib STATIC ${SPLINE_SOURCES})
target_include_directories(spline_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

# pybind11を使ったPythonモジュールの作成
if(BUILD_PYTHON_BINDINGS)
    pybind11_add_module(spline_module spline_py_module.cpp)
    target_link_libraries(spline_module PRIVATE spline_lib)
    target_include_directories(spline_module PRIVATE
        ${CMAKE_SOURCE_DIR}/include
    )

    # インストール設定をlib/pythonディレクトリに変更
    # これにより、ビルドディレクトリ内に生成されたモジュールファイルを手動でコピーできます
    set(PYTHON_MODULE_OUTPUT_DIR "${CMAKE_BINARY_DIR}")
    file(MAKE_DIRECTORY ${PYTHON_MODULE_OUTPUT_DIR})
    message(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
    set_target_properties(spline_module PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${PYTHON_MODULE_OUTPUT_DIR}
    )
endif()

# ライブラリのインストール設定
install(TARGETS spline_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
