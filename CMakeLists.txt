cmake_minimum_required(VERSION 3.16)

project(untitled VERSION 0.1 LANGUAGES CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable Qt automatic processing
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Find Qt6 packages
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets Gui)

# List all your sources, headers, UI files, and resources
set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    viewportobject.cpp
    viewportobject.h

)

# Create the executable (Qt6-style)
qt_add_executable(untitled
    ${PROJECT_SOURCES}
)

# Set FBX SDK path
# set(FBX_SDK_ROOT "C:/Program Files/Autodesk/FBX/FBX SDK/2020.3.7")

# Include directories
# include_directories(
#     "${FBX_SDK_ROOT}/include"
# )

# Link directories (fixed to match actual path)
# link_directories("${FBX_SDK_ROOT}/lib/x64/release")

# Link the required Qt and FBX SDK libraries
target_link_libraries(untitled PRIVATE
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Gui
    # "${FBX_SDK_LIB}"
    opengl32
)

# Optional: Windows/macOS bundle settings
set_target_properties(untitled PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)
