default: program
program:
	#gcc -o readdata readdata.c
	gcc -o gyro_accelerometer_incli -lm gyro_accelerometer_incli.c `sdl-config –cflags` `sdl-config –libs` -lSDL_image -lSDL_gfx -lSDL
clean:
	rm -f gyro_accelerometer_incli
