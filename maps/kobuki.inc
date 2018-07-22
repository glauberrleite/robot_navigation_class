define kobukilaser ranger (
	sensor (
		range [0 5]
		fov 360
		samples 1080
	)
	
	color "blue"
	size [0.1 0.1 0.1]
)


define kobuki position (
	bitmap "circle.png"
	color "black"
	drive "omni"
	gui_nose 1
	obstacle_return 1
	laser_return 1

	size [0.5 0.5 0.5]

	
	# When the center of rotation is offset from its center of area
	origin [0 0 0 0]

	kobukilaser ( pose [ 0 0 0 0 ] )
)


