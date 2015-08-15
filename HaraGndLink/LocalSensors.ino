void localsensors_process(void) {
	int current_pin = ACS715_PIN;
	int voltage_pin = VOLT_PIN;
	int curr_read; 
	int curr_out;
	int voltage_read;
	int voltage_out;
	
	/*	ACS715:
		sensor outputs about 100 at rest.
		Analog read produces a value of 0-1023, equating to 0v to 5v.
		"((long)sensorValue * 5000 / 1024)" is the voltage on the sensor's output in millivolts.
		There's a 500mv offset to subtract.
		The unit produces 133mv per amp of current, so divide by 0.133 to convert mv to ma
	*/
	curr_read = analogRead(current_pin);
//	curr_out = ( ((long)curr_read * 5000 / 1024) - 500 ) * 1000 / 133;
	curr_out = ( ((long)curr_read * 3300 / 1024) - 500 ) * 1000 / 133;			// Teensy analog pin just measures up to 3.3v, any voltage above that just gives max (=1024).
	current = (float) curr_out / 1000;
	
	// FrSky Voltage Sensor reading the "3S" option, where ratio is 6:1. Voltage range of 0..19.8 is scaled to 0..3.3
	voltage_read = analogRead(voltage_pin);
	voltage_out = ((long)voltage_read * 3300 / 1024) * 6;						// Multiply by divisor of voltage splitter, here it is 6
	voltage = (float) voltage_out / 1000;
}