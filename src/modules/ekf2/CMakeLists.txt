px4_add_module(
	MODULE modules__ekf2
	MAIN ekf2
	STACK_MAIN 2500
	SRCS
		ekf2_main.cpp
		ekf2_params.c
)
