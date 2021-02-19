#include "Quadcopter.h"

ErrorDetection::ErrorDetection() {

	ControlSystem controlSystem;

	pinMode(voltage_read_pin, INPUT);

	float _distanceE = controlSystem.getHSR05Error();
	float _pressureE = controlSystem.getBN055PressureError();
	float _gpsE = controlSystem.getGPSError();
	float _magE = controlSystem.getMagnetometerError();
	float _pololuPressureE = controlSystem.getPololuPressureError();
	float _MPU6050E_pitch, _MPU6050E_roll = controlSystem.getMPU6050Error();

}

ControlSystem::ControlSystem() {

	//Control
	s1_prop.attach(22);
	s2_prop.attach(23);
	s3_prop.attach(6);
	s4_prop.attach(5);

	//Legs
	pinMode(PY1, OUTPUT);
	pinMode(PY2, OUTPUT);

	//HSR05
	pinMode(trig_pin, OUTPUT);
	pinMode(echo_pin, INPUT);

	//GPS

	//Magnetometer

	//MPU6050

	//Pressure_Pololu && BN055
}

Buzzer::Buzzer() {
	pinMode(buzzer_pin, OUTPUT);
}

Led::Led(int r, int g, int b) {
	red_value = r;
	green_value = g;
	blue_value = b;

	pinMode(red_pin_led, OUTPUT);
	pinMode(green_pin_led, OUTPUT);
	pinMode(blue_pin_led, OUTPUT);
}

//method that handles all initialisations
void Initiliser::InitialiseSystem() {

	if (Serial.available() > 0) {
		state = Serial.readString();
	}


	switch (state) {

	case '1':
		buttonPressed = "PololuPressureButton";
		break;

	case '2':
		buttonPressed = "SD_CardButton";
		break;

	case '3':
		buttonPressed = "PropellersButton";
		break;

	case '4':
		buttonPressed = "GPSButton";
		break;

	case '5':
		buttonPressed = "HSR05Button";
		break;

	case '6':
		buttonPressed = "BN055Button";
		break;

	case '7':
		buttonPressed = "MPU6050Button";
		break;

	case '8':
		buttonPressed = "Leg_motorsButton";
		break;

	case '9':
		buttonPressed = "BMSButton";
		break;

	case ':':
		buttonPressed = "MagnetometerButton";
		break;

	default:
		break;

	}

	switch (buttonPressed) {

	case "PololuPressureButton":
		initialisePololuPressure();
		break;

	case "SD_CardButton":
		initialiseSD_Card();
		break;

	case "PropellersButton":
		initialisePropellers();
		break;

	case "GPSButton":
		initialiseGPS();
		break;

	case "HSR05Button":
		initialiseHSR05();
		break;

	case "BN055Button":
		initialiseBN055();
		break;

	case "MUP6050Button":
		initialiseMPU6050();
		break;

	case "BMSButton":
		initialiseBMS();
		break;

	case "Leg_motorsButton":
		initialiseLeg_motors();
		break;

	case "MagnetometerButton":
		initialiseMagnetometer();
		break;

	default:
		break;

	}
}

//methods to initialise each individual system
//done
void Initiliser::initialisePololuPressure() {

	if (ps.init())
	{
		ps.enableDefault();
		Serial.println("1");
	}
	else {
		Serial.println("0");
	}

}
//done
void Initiliser::initialiseSD_Card() {

	Serial.println("Initializing SD_Card");

	if (!SD.begin(10) {
		Serial.println("initialization failed!");
		while (1);
	}
	Serial.println("initialization done.");

	myFile = SD.open("test.txt", FILE_WRITE);

	if (myFile) {

		ControlSystem::getPololuPressureError();

		Serial.print("Writing to test.txt...");

		myFile.close();
		Serial.println("done.");
	}
	else {
		Serial.println("error opening test.txt");
	}
} 
Initiliser::initialisePropellers() {
	Serial.println("Initializing the Propellers");

	s1_prop.attach(22);
	s2_prop.attach(23);
	s3_prop.attach(6);
	s4_prop.attach(5);

	s1_prop.writeMicroseconds(1100);
	s2_prop.writeMicroseconds(1100);
	s3_prop.writeMicroseconds(1100);
	s4_prop.writeMicroseconds(1100);

	Serial.println("Confirm if Propeller initialisation was succesful");
}
//done
void Initiliser::initialiseGPS() {

	Serial.println("Initializing the GPS");

	bool newData = false;

	for (unsigned long start = millis(); millis() - start < 1000;)
	{
		while (ss.available())
		{
			char c = ss.read();
			// Serial.write(c); // uncomment this line if you want to see the GPS data flowing
			if (gps.encode(c)) // Did a new valid sentence come in?
				newData = true;
		}
	}

	if (newData)
	{
		float flat, flon;
		unsigned long age;

		Serial.println("Printing out NEMA sentence");

		gps.f_get_position(&flat, &flon, &age);
		Serial.print("LAT=");
		Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
		Serial.print(" LON=");
		Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
		Serial.print(" SAT=");
		Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
		Serial.print(" PREC=");
		Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
	}

	gps.stats(&chars, &sentences, &failed);
	Serial.print(" CHARS=");
	Serial.print(chars);
	Serial.print(" SENTENCES=");
	Serial.print(sentences);
	Serial.print(" CSUM ERR=");
	Serial.println(failed);
	if (chars == 0)
		Serial.println("** No characters received from GPS: check wiring **");

	Serial.println("Confirm if NEMA sentence is sensible");
}
//done
void Initiliser::initialiseHSR05() {
	Serial.println("Initializing the HSR05 sensor");

	float * distance = new float(0);
	float * duration = new float(0);

	for (int i = 0; i < 5; i++) {

		//trig_pin and echo_pin defined in Control System
		digitalWrite(trig_pin, LOW);
		delayMicroseconds(2);
		digitalWrite(trig_pin, HIGH);
		delayMicroseconds(10);
		digitalWrite(trig_pin, LOW);

		duration = pulseIn(echo_pin, HIGH);
		distance = duration / 29.1 / 2;

		Serial.println(distance);
	}
	delete(distance);
	delete(duration);

	Serial.println("Confirm if HSR05 data is sensible");
}
//done
void Initiliser::initialiseBN055() {
	Serial.println("Initializing the BN055 Pressure sensor");

	if (!bmp.begin()) {
		Serial.println("Failed to autodetect pressure sensor);
		while (1);
	}

	/* Default settings from datasheet. */
	bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
		Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
		Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
		Adafruit_BMP280::FILTER_X16,      /* Filtering. */
		Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
void Initiliser::initialiseMPU6050() {
	Serial.println("Initializing the MPU6050 sensor");

	sensor.initialize();

	Serial.println(sensor.testConnection() ? "Successfully Connected" : "Connection failed");
}
//done
void Initiliser::initialiseLeg_motors() {
	Serial.println("Testing the Leg_motors");

	digitalWrite(PY1, HIGH);       ///PY1 and PY2 define in ControlSystem class 
	digitalWrite(PY2, HIGH);

	Serial.println("Confirm leg motors are working");
	
}
//done
void Initiliser::initialiseBMS() {
	Serial.println("Testing the BMS readings");

	float* val = new int(0);

	val = analogRead(voltage_read_pin);

	Serial.println("Current voltage readign is ");
	Serial.println(val);
	
	Serial.println("Confirm if data is sensible");

	delete(val);
}
//done
void Initiliser::initialiseMagnetometer() {
	Serial.println("Initializing Magnetometer sensor");

	if (!mag.init())
	{
		Serial.println("Failed to autodetect pressure sensor!");
		while (1);
	}

	mag.enableDefault();
	Serial.println("Initialised succcesfully");
	Serial.println("Confirm if Magnetometer data is sensible");
} 

Control::Control(float usr_altitude, float height_ablove_objects, float usr_pitchAngle, float usr_rollAngle, float usr_yawAngle, float lat_coord, float lon_coord) {

	ControlSystem controlsystem;
	SystemStates systemstates;

	time = millis();

	defaultNEMASentence = systemstates.getCurrentNEMASentence();

	if (!Serial.read()(!usr_coord))
	{
		usr_coord = defaultNEMASentence;				//sets up default usr_coord value
	}

	if (!Serial.read()(!usr_yawAngle))
	{
		usr_yawAngle = systemstates.getCurrentBearing(usr_coord, defaultNEMASentence);				//sets up default usr_yawAngle value
	}

	pololuPressure_error = controlsystem.getPololuPressureError(usr_altitude);
	BMP280_error = controlsystem.getBMP280PressureError(usr_altitude);
	HSR05_error = controlsystem.getHSR05Error(height_ablove_objects);
	Magnetometer_error = controlsystem.getMagnetometerError(usr_yawAngle);
    Lat_error = controlsystem.getGPSError(lat_coord);
	Lon_error = controlsystem.getGPSError(lon_coord);
}

// runs system states
void SystemStates::Run() { //NB all that's happening is the user is setting their new desired error value

	if (Serial.available() > 0) 
		state = Serial.readString();

	if (!errordetection.flag) {

		switch (state) {
		case '1':
			StartFlight();
			break;

		case '2':
			Hover();
			break;

		case '3':
			Translate();
			break;

		case '4':
			endFlight();
			break;
		}
	}
	else
		endFlight();
}

//System States
void SystemStates::StartFlight() {

	current_altitude_error = 1;
	heigh_above_object = 0.5;
	pitchAngle = 0;
	rollAngle = 0;
	current_yaw_error = 0;
	current_coord = getCurrentNEMASentence();

	lat_coord = current_coord[0];    //default lat and lon coordinates
	lon_coord = current_coord[1];

	Control control(current_altitude_error, pitchAngle, rollAngle, current_yaw_error, current_coord);

	control.MMA();

}
void SystemStates::Hover() {

	current_altitude_error = ControlSystem::getPololuPressureError(); 
	heigh_above_object = ControlSystem::getHSR05Error();
	pitchAngle = 0;
	rollAngle = 0;
	current_yaw_error = ControlSystem::getMagnetometerError();
	current_coord = getCurrentNEMASentence();

	lat_coord = current_coord[0];    //default lat and lon coordinates
	lon_coord = current_coord[1];
	
	Control control(current_altitude_error, pitchAngle, rollAngle, current_yaw_error, lat_coord, lon_coord);

	control.MMA();

}
void SystemStates::Translate() {

	int StateOfQuartile = ControlSystem::GPSBearingQuaterile();

	bool DesiredQuatertile = ControlSystem::isDesiredQuaterile();

	current_coord = getCurrentNEMASentence();

	lat_coord = current_coord[0];    //default lat and lon coordinates
	lon_coord = current_coord[1];

	if (Serial.available() > 0) 
		usr_altitude = Serial.readString();      //defaultalt and height values in class.h

	if (Serial.available() > 0) 
		height_above_object = Serial.readString();

	//if (Serial.available() > 0)
	//	desired_yaw_angle = Serial.readString();
	//else
	desired_yaw_angle = getQuartileDesiredAngle();  //default yaw_angle

	if (Serial.available() > 0)
		lat_coord = Serial.readString();

	if (Serial.available() > 0)
		lon_coord = Serial.readString();
		

	pitchAngle = 0;    //default pitch and roll angle
	rollAngle = 0;

	Control control(usr_altitude, height_above_object, pitchangle, rollangle, desired_yaw_angle, lat_coord, lon_coord);

	control.MMA();
		
	if (DesiredQuatertile) {

		usr_yawangle = serial.read();
		usr_coord = serial.read(); //user passes in an array of lat, lon

		Control control(usr_altitude, height_above_object, pitchangle, rollangle, usr_yaw_angle, usr_coord);  //pitchAngle

		control.MMA();
	}

}
int SystemStates::endFlight() {
	float speed_loss = 0;
	float time = 0;

	float pwm_s1, float pwm_s2, float pwm_s3, float pwm_s4 = Control::MMA();

	ControlSystem::eject();

	if (pwm_s1 != 1000 && pwm_s2 != 1000 && pwm_s3 != 1000 && pwm_s4 != 1000)
	{
		speed_loss += exp(2 * time);
		PID_s1 -= speed_loss;
		PID_s2 -= speed_loss;
		PID_s3 -= speed_loss;
		PID_s4 -= speed_loss;

		s1_prop.writeMicroseconds(PID_s1);
		s2_prop.writeMicroseconds(PID_s2);
		s3_prop.writeMicroseconds(PID_s3);
		s4_prop.writeMicroseconds(PID_s4);

		time += 0.01;

	}
	Buzzer buzzer;
	buzzer.powerOff();
	return 0;
}

//Bearing value to be executed to the error method 

float SystemStates::getCurrentBearing(float usr_coord[], float current_coord[] ) {  //usr_coord can be seen as the desired_coord

	float lat1 = usr_coord[0];
	float lat2 = current_coord[0];
	float long1 = usr_coord[1];
	float long2 = current_coord[1];

	float change_in_long = long2 - long1;
																						//FORMUAL USED	
	float X = cos(lat2) * sin(change_in_long);											//Let ‘R’ be the radius of Earth,
																						//‘L’ be the longitude,
	float Y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(change_in_long);		//‘θ’ be latitude,
																						//‘β‘ be Bearing.
	bearing = atan2(X, Y);																//β = atan2(X, Y)
																						//X = cos θb * sin ∆L
	return bearing;																		//Y = cos θa * sin θb – sin θa * cos θb * cos ∆L
																
}

//GPS current NEMA sentence
float* SystemStates::getCurrentNEMASentence() {

	bool newData = false;
	float* NEMA_Sentence_ptr;

	while (!newData)
	{
		for (unsigned long start = millis(); millis() - start < 1000;)
		{
			while (ss.available())
			{
				char c = ss.read();
				// Serial.write(c); // uncomment this line if you want to see the GPS data flowing
				if (gps.encode(c)) // Did a new valid sentence come in?
					newData = true;
			}
		}

		if (newData)
		{

			float flat, flon;
			unsigned long age;

			Serial.println("Printing out NEMA sentence");

			gps.f_get_position(&flat, &flon, &age);
			Serial.print("LAT=");
			Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
			Serial.print(" LON=");
			Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
			Serial.print(" SAT=");
			Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
			Serial.print(" PREC=");
			Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

			NEMA_Sentence[0] = flat;
			NEMA_Sentence[1] = flon;
			NEMA_Sentence[2] = age;

			NEMA_Sentence_ptr = NEMA_Sentence;

		}
	}

	return NEMA_Sentence_ptr;

}

//MMA
void Control::MMA() {

	float PololuPressure_PID = getPololuPressurePID_Values();
	float Mag_PID = getMagnetometerPID_Values();
	float BN055_PID = getBN055PressurePID_Values();
	float GPS_Lat_PID = getGPSPID_Lat_Values();
	float GPS_Long_PID = getGPSPID_Long_Values();
	float MPU6050_Pitch_PID = getMPU6050PID_Pitch_Values();
	float MPU6050_Roll_PID = getMPU6050PID_Roll_Values();
	float HSR05_PID = getHSR05PID_Values();

	float THRUSTcmd = (PololuPressure_PID + BN055_PID + HSR05_PID) / 3; // dividing by 3 to normalize THRUSTcmd. 3 because of the three 
																		  //  inputs into THRUSTcmd

	float PITCHcmd = MPU6050_Pitch_PID - GPS_Lat_PID;
	
	float ROLLcmd = MPU6050_Roll_PID - GPS_Long_PID;

	float YAWcmd = Mag_PID;

	PID_s1 = THRUSTcmd + YAWcmd + PITCHcmd + ROLLcmd;      // s1     s3   //s1 must spin anti-clockwise
	PID_s2 = THRUSTcmd - YAWcmd - PITCHcmd + ROLLcmd;      //   O   O
	PID_s3 = THRUSTcmd - YAWcmd + PITCHcmd - ROLLcmd;      //    \-/
	PID_s4 = THRUSTcmd + YAWcmd - PITCHcmd - ROLLcmd;      //    /-\
														   //   O   O
	                                                       // s2     s4

	pwm_s1 = throttle + PID_s1;
	pwm_s2 = throttle + PID_s2;
	pwm_s3 = throttle + PID_s3;
	pwm_s4 = throttle + PID_s4;

	s1_prop.writeMicroseconds(pwm_s1);
	s2_prop.writeMicroseconds(pwm_s2);
	s3_prop.writeMicroseconds(pwm_s3);
	s4_prop.writeMicroseconds(pwm_s4);

	return pwm_s1, pwm_s2, pwm_s3, pwm_s4;
}

//methods to calculate the 4 actuator speeds of each system
float Control::getPololuPressurePID_Values() {

	timePrev = time;
	time = millis();
	elapsedTime = (time - timePrev) / 1000;

	float kp = 3.55;
	float ki = 0.005;
	float kd = 2.05;

	pid_p = kp * pololuPressure_error;

	if (-0.2 < pololuPressure_error and error < pololuPressure_error)
		pid_i_pololu = pid_i_pololu + (ki * pololuPressure_error);

	pid_d = kd * ((pololuPressure_error - previous_error) / elapsedTime);

	PID = pid_p + pid_i_pololu + pid_d;

	previous_error = pololuPressure_error;
    
	return PID;
}
float Control::getBMP280ressurePID_Values() {

	timePrev = time;
	time = millis();
	elapsedTime = (time - timePrev) / 1000;

	float kp = 3.55;
	float ki = 0.005;
	float kd = 2.05;

	pid_p = kp * BMP280_error;

	if (-0.2 < BMP280_error and error < BMP280_error)
		pid_i_BMP280 = pid_i_BMP280 + (ki * BMP280_error);

	pid_d = kd * ((BMP280_error - previous_error) / elapsedTime);

	PID = pid_p + pid_i_BMP280 + pid_d;

	previous_error = BMP280_error;

	return PID;
}
float Control::getGPSPID_Lat_Values() {  

	timePrev = time;
	time = millis();
	elapsedTime = (time - timePrev) / 1000;

	float kp = 3.55;
	float ki = 0.005;
	float kd = 2.05;
	
	//PID for Lat
	pid_p = kp * Lat_error;

	if (-0.2 < Lat_error && Lat_error < 0.2)
		pid_i = pid_i_Lat_GPS + (ki * Lat_error);

	pid_d = kd * ((Lat_error - previous_error) / elapsedTime);

	Lat_PID = pid_p + pid_i_Lat_GPS + pid_d;

	previous_lat_error = Lat_error;

	return PID;
}
float Control::getGPSPID_Long_Values() {

	// this method is going to get GPS Latitutde and Longitude values

	timePrev = time;
	time = millis();
	elapsedTime = (time - timePrev) / 1000;

	float kp = 3.55;
	float ki = 0.005;
	float kd = 2.05;

	//PID for Long
	pid_p = kp * Long;

	if (-0.2 < Lon_error && Lon_error < 0.2)
		pid_i = pid_i_Long_GPS + (ki * Long_error);

	pid_d = kd * ((Lon_error - previous_error) / elapsedTime);

	Long_PID = pid_p + pid_i_Long_GPS + pid_d;

	previous_lon_error = Lon_error;

	return PID;
}
float Control::getMPU6050PID_Pitch_Values() {

	timePrev = time;
	time = millis();
	elapsedTime = (time - timePrev) / 1000;

	float kp = 3.55;
	float ki = 0.005;
	float kd = 2.05;

	pid_p = kp * MPU6050_error_Pitch;

	if (-3 < MPU6050_error_Pitch < 3)
	{
		pid_i = pid_i + (ki * MPU6050_error_Pitch);
	}

	pid_d = kd * ((MPU6050_error_Pitch - previous_error) / elapsedTime);

	PID = pid_p + pid_i + pid_d;

	previous_error = MPU6050_error_Pitch;

	return PID;
}
float Control::getMPU6050PID_Roll_Values() {

	timePrev = time;
	time = millis();
	elapsedTime = (time - timePrev) / 1000;

	float kp = 3.55;
	float ki = 0.005;
	float kd = 2.05;

	pid_p = kp * MPU6050_error_Roll;

	if (-3 < MPU6050_error_Pitch < 3)
	{
		pid_i = pid_i + (ki * MPU6050_error_Roll);
	}

	pid_d = kd * ((MPU6050_error_Roll - previous_error) / elapsedTime);

	PID = pid_p + pid_i + pid_d;

	previous_error = MPU6050_error_Roll;

	return PID;
}
float Control::getMagnetometerPID_Values() {

	timePrev = time;
	time = millis();
	elapsedTime = (time - timePrev) / 1000;

	float kp = 3.55;
	float ki = 0.005;
	float kd = 2.05;

	pid_p = kp * Magnetometer_error;

	if (-0.2 < Magnetometer_error and Magnetometer_error < 0.2)
		pid_i_Magnetometer = pid_i_Magnetometer + (ki * Magnetometer_error);

	pid_d = kd * ((Magnetometer_error - previous_error) / elapsedTime);

	PID = pid_p + pid_i_Magnetometer + pid_d;

	previous_error = Magnetometer_error;

	return PID;
}
float Control::getHSR05PID_Values() {
	// NB. no I value in PID controller for HSR05 is needed because 

	timePrev = time;
	time = millis();
	elapsedTime = (time - timePrev) / 1000;

	float kp = 3.55;
	float ki = 0.005;
	float kd = 2.05;

	pid_p = kp * HSR05_error;

	if (-0.2 < HSR05_error and HSR05_error < 0.2)
		pid_d_HSR05 = pid_d_HSR05 + (ki * HSR05_error);

	pid_d = kd * ((HSR05_error - previous_error) / elapsedTime);

	PID = pid_p + pid_d_HSR05 + pid_d;

	previous_error = HSR05_error;

	return PID;
}
float Control::rotateToQuaterile() {

}

// Control System
void ControlSystem::eject()
{
	digitalWrite(PY1, HIGH);
	digitalWrite(PY2, HIGH);
}
float ControlSystem::getPololuPresssureError(float desired_altitude)
{
	pressure = ps.readPressureMillibars();
	altitude = 1490 - ps.pressureToAltitudeMeters(pressure);

	error = desired_altitude - altitude;

	return error;
}
float ControlSystem::getHSR05Error(float desired_distance)
{

	digitalWrite(trig_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(trig_pin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig_pin, LOW);

	duration = pulseIn(echo_pin, HIGH);
	distance = duration / 29.1 / 2;

	error = desired_distance - distance;

	return error;
}
float ControlSystem::getBMP280PressureError(float desired_altitude) {
	pressure = bmp.readAltitude(myAltitude_above_sea_level);

	error = desired_altitude - pressure;

}
float * ControlSystem::getGPSError(float* desired_coord) {

	bool newData = false;

	while (!newData)
	{
		for (unsigned long start = millis(); millis() - start < 1000;)
		{
			while (ss.available())
			{
				char c = ss.read();
				// Serial.write(c); // uncomment this line if you want to see the GPS data flowing
				if (gps.encode(c)) // Did a new valid sentence come in?
					newData = true;
			}
		}

		if (newData)
		{
			float flat, flon;

			if ((StateOfQuartile = 0 && desired_yaw_angle = 90) || (StateOfQuartile = 1 && desired_yaw_angle = 180)   /       //this logic inverts gps lat error so copter can 
				|| (StateOfQuartile = 2 desired_yaw_angle = 270) || (StateOfQuartile = 3 && desired_yaw_angle = 0))             // roll left as well
			//GPS_Lat_Error = GPS_Lat_Error * -1;

			gps.f_get_position(&flat, &flon);

			NEMA_Sentence[0] = flat;
			NEMA_Sentence[1] = flon;

			int number_of_nema_elements = (sizeof(NEMA_Sentence) / sizeof(NEMA_Sentence[0]);

			for (int i = 0; i < number_of_nema_elements; i++) {
				error[i] = desired_coord[i] - NEMA_Sentence[i];
			}
		}

	}

	return  error;
}
float ControlSystem::getMPU6050Error(float* pitch_error, float* roll_error) {

	sensor.getMotion6(&Acc_rawX, &Acc_rawY, &Acc_rawZ, &Gyr_rawX, &Gyr_rawY, &Gyr_rawZ);

	Acc_rawY = map(Acc_rawY, -17000, 17000, 0, 180);
	Acc_rawX = map(Acc_rawX, -17000, 17000, 0, 180);
	Acc_rawZ = map(Acc_rawZ, -17000, 17000, 0, 180);

	Acceleration_angle[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;
	Acceleration_angle[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;


	Gyro_angle[0] = Gyr_rawX / 131.0;
	Gyro_angle[1] = Gyr_rawY / 131.0;

	Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0]; // X-Axis
	Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acceleration_angle[1]; // Y-Axis

	*pitch_error = desired_pitch_angle - Total_angle[0];
	*roll_error = desired_roll_angle - Total_angle[1];

}
float ControlSystem::getCopterBearing() {
	mag.read();

	xGaussData = mag.m.x;
	yGaussData = mag.m.y;

	if (xGaussData == 0) {
		if (yGaussData < 0)
		{
			d = 90;
		}
		else
			d = 0;
	}
	else
		d = atan2(yGaussData, xGaussData) * (rad_to_deg);

	if (d > 360)
		d = d - 360;
	else if (d < 0)
		d = d + 360;

	return d;
}
float ControlSystem::getMagnetometerError(float desired_yaw_angle) {

	float d = getCopterBearing();

	error = desired_yaw_angle - d;

	return error;
}
float ControlSystem::GPSBearingQuaterile() {


	Quaterile StateOfQuaterile;
	
	float GPSDiresctionBearing = getCurrentBearing();

	if (GPSDiresctionBearing >= 0 and GPSDiresctionBearing < 90)
		StateOfQuaterile = Q1;

	if (GPSDiresctionBearing >= 90 and GPSDiresctionBearing < 180)
		StateOfQuaterile = Q2;

	if (GPSDiresctionBearing >= 180 and GPSDiresctionBearing < 270)
		StateOfQuaterile = Q3;

	if (GPSDiresctionBearing >= 270 and GPSDiresctionBearing < 0)
		StateOfQuaterile = Q4;

	return StateOfQuaterile;
}
float ControlSystem::CopterDirectionQuaterile() {


	Quaterile StateOfQuaterile;

	float CopterDirectionBearing = getCopterBearing();

	if (CopterDirectionBearing >= 0 && CopterDirectionBearing < 90)
		StateOfQuaterile = Q1;

	if (CopterDirectionBearing >= 90 && CopterDirectionBearing < 180)
		StateOfQuaterile = Q2;

	if (CopterDirectionBearing >= 180 && CopterDirectionBearing < 270)
		StateOfQuaterile = Q3;

	if (CopterDirectionBearing >= 270 && CopterDirectionBearing < 0)
		StateOfQuaterile = Q4;

	return StateOfQuaterile;
}
bool ControlSystem::isDesiredQuaterile() {
	
	float GPS_Quartile = GPSBearingQuaterile();
	float Copter_Quartile = CopterDirectionQuaterile();

	if (GPS_Quartile == Copter_Quartile)
		return true;

	return false;
}

// Buzzer
void Buzzer::powerOn() {

  tone(buzzer_pin, E);
  delay(200);
  noTone(buzzer_pin);

  tone(buzzer_pin, F_sharp);
  delay(200);
  noTone(buzzer_pin);

  tone(buzzer_pin, G);
  delay(200);
  noTone(buzzer_pin);

  tone(buzzer_pin, A);
  delay(500);
  noTone(buzzer_pin);
}

void Buzzer::powerOff(){
  
  tone(buzzer_pin, A);
  delay(200);
  noTone(buzzer_pin);

  tone(buzzer_pin, G);
  delay(200);
  noTone(buzzer_pin);

  tone(buzzer_pin, F_sharp);
  delay(200);
  noTone(buzzer_pin);

  tone(buzzer_pin, E);
  delay(500);
  noTone(buzzer_pin);
}

// Led
void Led::blinkLed() {

	analogWrite(red_pin_led, red_value);
	analogWrite(green_pin_led, green_value);
	analogWrite(blue_pin_led, blue_value);
}

// Bluetooth
char Bluetooth::getInput() {

	if (Serial.available() > 0) 
		state = Serial.read(); 

	return state;
}


// ErrorDetection
bool ErrorDetection::flag() {

	bool check1 = batteryVoltageReading();
	bool check2 = HSR05_error();
	bool check3 = BN055_error();
	bool check4 = GPS_error();
	bool check5 = MPU6050_error();
	bool check6 = PololuPressure_error();
	bool check7 = Magnetometer_error();

	if (check1 || check2 || check3 || check4 || check5 || check6 || check7)
		return true; // will begin endFlight() in Run()

	return false; 
}

bool ErrorDetection::batteryVoltageReading() {
	val = analogRead(voltage_read_pin);

	if (val < 3.3)
		return true;
	return false;
}
bool ErrorDetection::MPU6050_error() {

	if (_MPU6050E_pitch  > 50 || _MPU6050E_pitch < -50)
		return true;	 
						 
	if (_MPU6050E_roll  > 50 || _MPU6050E_roll < -50)
		return true;	 
	return false;
}
bool ErrorDetection::HSR05_error() {

	if (_distanceE > 3.3)
		return true;
	return false;

}
bool ErrorDetection::BMP280_error() {

	if (_pressureE > 3.3)
		return true;
	return false;

}
bool ErrorDetection::GPS_error() {

	if (_gpsE > 3.3)
		return true;
	return false;

}
bool ErrorDetection::Magnetometer_error() {

	if (_magE > 3.3)
		return true;
	return false;

}
bool ErrorDetection::PololuPressure_error() {

	if (_pololuPressureE > 3.3)
		return true;
	return false;

}









