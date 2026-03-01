find_package(Eigen3 REQUIRED)
find_package(Pagmo REQUIRED)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

add_library(gravitas SHARED
        src/physics/rigidbody.cpp
        include/gravitas/physics/rigidbody.h
        src/physics/kepler.cpp
        include/gravitas/physics/kepler.h
        src/numerics.cpp
        include/gravitas/numerics.h
        src/data.cpp
        include/gravitas/data.h
)
target_include_directories(gravitas PUBLIC include)
set_target_properties(gravitas PROPERTIES PUBLIC_HEADER include)
target_link_libraries(gravitas PUBLIC Pagmo::pagmo nlohmann_json::nlohmann_json)
