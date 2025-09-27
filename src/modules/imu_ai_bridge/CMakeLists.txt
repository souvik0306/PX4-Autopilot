px4_add_module(
    MODULE modules__imu_ai_bridge
    MAIN imu_ai_bridge
    SRCS
        imu_ai_bridge.cpp
    DEPENDS
        platforms__common
)
