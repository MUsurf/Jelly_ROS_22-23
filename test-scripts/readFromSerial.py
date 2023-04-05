import serial

# Start the serial port
serPort = serial.Serial('/dev/ttyACM0')

try:
	while(True):
		# Read the line of depth data. type(depth) == 'bytes'
		depth = serPort.readline()
		
		# Convert the read value into a string
		depthString = depth.decode("utf-8")
		
		# Print the string without a newline at the end
		print(depthString, end ="")
		
except KeyboardInterrupt:
	print("Closing serial port")
	serPort.close()
	print("Done.")
