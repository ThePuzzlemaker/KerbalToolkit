set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        Svg
        SvgWidgets
        REQUIRED)

add_executable(kerbtk
        src/main.cpp
        src/widgets/colors.h
        src/widgets/floating_window.cpp
        src/widgets/floating_window.h
        src/widgets/icon.h
        src/widgets/icon.cpp
        src/widgets/time.cpp
        src/widgets/time.h
        src/widgets/util.h
        src/displays/time_utils.cpp
        src/displays/time_utils.h
)
target_link_libraries(kerbtk
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::Svg
        Qt::SvgWidgets
        gravitas)

if (NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/assets")
    file(DOWNLOAD "https://phosphoricons.com/assets/phosphor-icons.zip" "${CMAKE_CURRENT_BINARY_DIR}/phosphor-icons.zip")
    file(ARCHIVE_EXTRACT INPUT "${CMAKE_CURRENT_BINARY_DIR}/phosphor-icons.zip" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/assets/" PATTERNS "Fonts/**/*.ttf" "LICENSE")
    file(RENAME "${CMAKE_CURRENT_BINARY_DIR}/assets/Fonts" "${CMAKE_CURRENT_BINARY_DIR}/assets/icons")
endif ()

file(GLOB_RECURSE BINARY_RESOURCES_FILES "${CMAKE_CURRENT_BINARY_DIR}/assets/*")

qt_add_resources(kerbtk "resources"
        PREFIX "/"
        FILES
        assets/fonts/InterVariable.ttf
        assets/fonts/InterVariable-Italic.ttf
        assets/fonts/LICENSE-Inter.txt
)
qt_add_resources(kerbtk "buildResources"
        PREFIX "/"
        BASE "${CMAKE_CURRENT_BINARY_DIR}/"
        FILES
        ${BINARY_RESOURCES_FILES}
)
