Serial Data Collection steps:

1. Run only the python program after uploading the code to the Arduino. Change the following:
- Add the correct port. On Mac, it will be in the form "/dev/cu.usbmodemXXXX" and in Windows it will be in the form "COMX"

2. Remember to check if each sensor is on, and see the first few rows within the CSV file generated in the directory for potential readings.
3. Set frequency of data collection in python script for more/less data points read from the sensors.
