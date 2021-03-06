define kobukilaser ranger (
	sensor (
		range [0 5]
		fov 270
		samples 880
	)
	
	color "blue"
	size [0.5 0.5 0.1]
)


define kobuki position (
	bitmap "circle.png"
	color "black"
	drive "diff"
	gui_nose 1
	obstacle_return 1
	laser_return 1

	size [0.5 0.5 0.5]
	
	# four DOF kinematics limits
	# [ xmin xmax ymin ymax zmin zmax amin amax ]	
	velocity_bounds     [-1 1 0 0 0 0 -90.0 90.0 ]
	acceleration_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]
	
	# When the center of rotation is offset from its center of area
	origin [0 0 0 0]

	kobukilaser ( pose [ 0 0 0 0 ] )
)


