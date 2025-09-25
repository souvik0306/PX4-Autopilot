px4_add_module(
	MODULE modules__imu_ai_bridge
	MAIN imu_ai_bridge
	COMPILE_FLAGS
		-Wno-cast-align
	SRCS
		imu_ai_bridge.cpp
)
