# the simulation engine of the CAM simulator does not depend on OpenGL, test it directly, with the
# accuracy validation against the exact material of a job
set(CAMSimulatorEngine_DIR ${CMAKE_SOURCE_DIR}/src/Mod/CAM/PathSimulator/AppGL)
add_library(CAMSimulatorAccuracy STATIC
    SimulatorAccuracy.cpp
    SimulatorAccuracy.h
    ${CAMSimulatorEngine_DIR}/GCodeParser.cpp
    ${CAMSimulatorEngine_DIR}/MachineKinematics.cpp
    ${CAMSimulatorEngine_DIR}/MaterialGrid.cpp
    ${CAMSimulatorEngine_DIR}/MaterialJob.cpp
    ${CAMSimulatorEngine_DIR}/MaterialLayout.cpp
    ${CAMSimulatorEngine_DIR}/MaterialMesher.cpp
    ${CAMSimulatorEngine_DIR}/MaterialModel.cpp
    ${CAMSimulatorEngine_DIR}/MaterialSimulation.cpp
    ${CAMSimulatorEngine_DIR}/MaterialTool.cpp
    ${CAMSimulatorEngine_DIR}/ThreadPool.cpp
    ${CAMSimulatorEngine_DIR}/ToolpathCheck.cpp
)
target_include_directories(CAMSimulatorAccuracy PUBLIC ${CMAKE_SOURCE_DIR}/src)
find_package(Threads REQUIRED)
target_link_libraries(CAMSimulatorAccuracy PUBLIC Threads::Threads)

add_executable(CAM_tests_run
    MaterialAccuracy.cpp
    MaterialSimulator.cpp
    Path.cpp
    SimulatorGCode.cpp
    SimulatorMachine.cpp
)
target_link_libraries(CAM_tests_run CAMSimulatorAccuracy)

# accuracy of a job written by the GUI, see src/Mod/CAM/PathSimulator/Benchmark/README.md
add_executable(CAM_simulator_accuracy SimulatorAccuracyTool.cpp)
target_link_libraries(CAM_simulator_accuracy CAMSimulatorAccuracy)
if(WIN32)
    set_target_properties(CAM_simulator_accuracy PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
else()
    set_target_properties(CAM_simulator_accuracy PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
endif()
