project("ApproxMVBBExample-MVBB2")

include(SetTargetCompileOptions)

add_executable(${PROJECT_NAME}  src/main.cpp )
setTargetCompileOptions(ApproxMVBBExample-MVBB)

target_link_libraries(${PROJECT_NAME} "ApproxMVBB::Core")
