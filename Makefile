prereq:;
	@-test -e lib/arduino-MLX90393 || echo "Error: clone github.com/pastcompute/arduino-MLX90393.git#fix_for_xmc1100 to lib/arduino-MLX90393"
	@-test -e lib/sx1276-arduino || echo "Error: clone github.com/pastcompute/sx1276-arduino.git to lib/sx1276-arduino"
	@test -e lib/arduino-MLX90393 && test -e lib/sx1276-arduino



