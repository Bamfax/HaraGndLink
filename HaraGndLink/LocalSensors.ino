void localsensors_process(void) {
	int current_pin = ACS715_PIN;
	int sensorValue; 
	int outputValue;
	
	sensorValue = analogRead(current_pin);
//	outputValue = ( ((long)sensorValue * 5000 / 1024) - 500 ) * 1000 / 133;
	outputValue = ( ((long)sensorValue * 3300 / 1024) - 500 ) * 1000 / 133;			// Teensy analog pin just measures up to 3.3v, any voltage above that just gives max (=1024).
	current = (float) outputValue / 1000;
	
	/*	sensor outputs about 100 at rest.
		Analog read produces a value of 0-1023, equating to 0v to 5v.
		"((long)sensorValue * 5000 / 1024)" is the voltage on the sensor's output in millivolts.
		There's a 500mv offset to subtract.
		The unit produces 133mv per amp of current, so divide by 0.133 to convert mv to ma */
}