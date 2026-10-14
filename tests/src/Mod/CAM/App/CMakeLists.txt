add_executable(CAM_tests_run
    Path.cpp
    Simulator.cpp
)

# the simulation engine of the CAM simulator does not depend on OpenGL, test it directly
set(CAMSimulatorEngine_DIR ${CMAKE_SOURCE_DIR}/src/Mod/CAM/PathSimulator/AppGL)
target_sources(CAM_tests_run PRIVATE
    ${CAMSimulatorEngine_DIR}/CutterProfile.cpp
    ${CAMSimulatorEngine_DIR}/DexelLayout.cpp
    ${CAMSimulatorEngine_DIR}/DexelMesher.cpp
    ${CAMSimulatorEngine_DIR}/DexelSimulation.cpp
    ${CAMSimulatorEngine_DIR}/DexelStock.cpp
    ${CAMSimulatorEngine_DIR}/ThreadPool.cpp
)
