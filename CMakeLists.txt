cmake_minimum_required(VERSION 3.22)

# 仅生成一次，用户可自由修改
# 核心修改：补全-march、规范参数配置、修正Include目录、添加调试/优化

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME watercar)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# ========================================
# 【STM32F407 专属修正：指令集 + 浮点ABI + 链接参数】
# ========================================
# 1. 完整的CPU/架构参数（补全-march=armv7e-m，解决核心报错）
set(CPU_PARAMETERS
    -mcpu=cortex-m4          # Cortex-M4F内核
    -march=armv7e-m          # 关键：补全M4对应的ARM架构版本（解决-march参数缺失）
    -mthumb                  # Thumb-2指令集（STM32F4必选）
    -mthumb-interwork        # 兼容Thumb/ARM互操作
    -mfloat-abi=hard         # 硬浮点ABI（F4带FPU必须）
    -mfpu=fpv4-sp-d16        # F4的FPU型号（固定值）
)

# 2. 编译参数：分Debug/Release配置（调试+优化）
set(COMPILE_OPTIONS
    ${CPU_PARAMETERS}
    -ffunction-sections      # 函数分段（便于链接时GC）
    -fdata-sections          # 数据分段
    -fno-strict-aliasing     # 关闭严格别名检查
    -fno-builtin             # 禁用内置函数
    -Wall                    # 开启基础警告（可选，便于调试）
    -Wextra                  # 开启额外警告（可选）
)

# Debug模式：调试信息+无优化；Release模式：优化+少调试信息
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND COMPILE_OPTIONS -g3 -O0)  # -g3：完整调试信息，-O0：无优化
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    list(APPEND COMPILE_OPTIONS -O2 -g1)  # -O2：中度优化，-g1：精简调试信息
endif()

# 3. 链接参数：简化+补全架构，移除冗余
set(LINK_OPTIONS
    -lnosys                  # 无操作系统支持
    -Wl,--gc-sections        # 移除未使用的段（减固件大小）
    -Wl,--print-memory-usage # 打印内存使用（便于调试）
    -u _printf_float         # 支持浮点打印（printf输出float）
    ${CPU_PARAMETERS}        # 链接阶段复用CPU参数（确保架构匹配）
)
# ========================================

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# 【关键修正】现代CMake规范：目标级参数（替代全局CMAKE_*_FLAGS）
# 编译参数（C/ASM）
target_compile_options(${CMAKE_PROJECT_NAME}
    PRIVATE
    ${COMPILE_OPTIONS}
    $<$<COMPILE_LANGUAGE:ASM>:${CPU_PARAMETERS}>  # 汇编文件单独指定CPU参数
)

# 链接参数
target_link_options(${CMAKE_PROJECT_NAME}
    PRIVATE
    ${LINK_OPTIONS}
)

# Link directories setup（按需添加库搜索路径，当前无则保留空）
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/bsp_encoder.c
    Core/Src/bsp_key.c
    Core/Src/bsp_motor.c
    Core/Src/mecanum.c
    Core/Src/odom.c
    Core/Src/trajectory.c
    Core/Src/pid.c
    # Add user sources here
)

# 【关键修正】Include目录：指定目录而非具体.h文件
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc  # 所有Core/Inc下的头文件都能被找到，无需逐个列.h
    # Add user defined include paths（如其他模块目录）
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols（如：STM32F407xx，CubeMX通常已自动加）
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)

# 【可选】生成bin/hex文件（便于烧录）
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating bin/hex files for ${CMAKE_PROJECT_NAME}"
)