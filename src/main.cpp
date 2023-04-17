/*

  "▀▓╣╣╣▓▓▓▓▄▄▄╖,
	 ╙▓╣╣╣╣╣╣╣╣╣╣╣╣▓▓▄▄╖,,
	   ╙▓╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣▓▓▓▓▓▓▄▄▄▄▄▄▄▄▄▄╖╖╖╖╖╖╖╖╖╖,,,,,
		 ▀╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣▓▓▓▄▄╖,
		  ╙╣▓▓▓▓▓╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣▓▓▄,
		   ╓▄▓▓▓▄▄┌╙▓╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣▓▄,
		 ▄╣╣╣╣╣╣╣╣╣▓,╙╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣▓,
		▓╣╣╣╣╣╣╣╣╣╣╣╣ ╘╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣▓
		╣╣╣╣╣╣╣╣╣╣╣╣╣L ╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣
		╚╣╣╣╣╣╣╣╣╣╣╣▓ ,╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣╣
		 ╙▓╣╣╣╣╣╣╣▓▀                                                    ╚▓▓▓▓▓▓▓
			'╙"""                                                         "▀▀▀`

////////////////////////////////////////////////////////////////////////////////////////////////
//                                        DROMEAS                                             //
//                                 LINE FOLLOWING PROJECT                                     //
//                          Robotics Club - University of Patras                              //
//                             http://robotics-club.upatras.gr/                               //
//                          https://www.facebook.com/Polymechanon/                            //
//                                                                                            //
// - Microcontroller: Teensy 3.2                                                              //
// - Motor Driver   : TB6612FNG(Pololu)                                                       //
// - Line Sensors   : QTR-8RC or QTR-8A(Pololu)                                               //
// - Line Sensors Library: QTRSensors.h from Pololu modified to work with Teensy              //
//                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////

*/

#include "Arduino.h"
#include "QTRSensors.h"
#include "config.h"

// QTR sensor object
QTRSensorsRC qtrrc((unsigned char[]){pin_QTR_1, pin_QTR_2, pin_QTR_3, pin_QTR_4, pin_QTR_5, pin_QTR_6, pin_QTR_7, pin_QTR_8}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); // QTR digital
// QTRSensorsAnalog qtrrc((unsigned char[]) {pin_QTR_1 , pin_QTR_2 , pin_QTR_3 , pin_QTR_4 , pin_QTR_5 , pin_QTR_6, pin_QTR_7, pin_QTR_8}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_pin); //QTR analog

//-----------------------ALGORITHM VARIABLES-----------------------------//

unsigned long previousTime = 0, currentTime; // To keep track of actual loop time.

unsigned int sensorValues[NUM_SENSORS]; // Sensors' calibrated values 0-1000.

int sensorColour[NUM_SENSORS]; // Sensors' read colours  0-1.

bool online[NUM_SENSORS]; // How many sensors see line.
bool flagOnLine = false; // False: No sensor sees line, True: At least one sensor sees line.

double pidErrorDerivative = 0.0;

// pid - angle decision {[-350,350], 1000, -1000}
int reflectanceOutput = 0;

int flagAngleDetection; // When an angle is detected.

int pidError = 0; // Error passed into PID controller.
int pidErrorPrev = 0; // Previous loop PID error (used for derivative calculation).
int pidErrorSum = 0; // Sum of all previous errors (used for integral calculation).

int linearVel, angularVel;
static int lastValue = 0; // Initially assume that the line is on the left.
int pwmLeft = 0, pwmRight = 0;

//-----------------------------------FUNCTIONS-------------------------------------//

// Read Sensors: Function used to read the value of each sensor from the object and normalise them between 0 and 1000
void readSensors()
{ // read from all sensors

    // read reflectance sensors
    qtrrc.read(sensorValues); // get new values
    for (unsigned int i = 0; i < NUM_SENSORS; i++) {
        // Map raw sensor values into a given range (0-1000).
        sensorColour[i] = map(sensorValues[i], qtrrc.calibratedMinimumOn[i] + 20, qtrrc.calibratedMaximumOn[i], 0, 1000);
        if (sensorColour[i] < 0)
            sensorColour[i] = 0;
        if (sensorColour[i] > 1000)
            sensorColour[i] = 1000;
    }
}

// checkOnLine: Function used to check if robot is over line
void checkOnLine()
{
    for (unsigned int i = 0; i < NUM_SENSORS; i++) { // Loop through each sensor
        int value = sensorColour[i]; // Read the value of each sensor
        // keep track of whether we see the line at all
        if (value > 400) {
            flagOnLine = true;
            online[i] = true;
        }
        else {
            online[i] = false;
        } // end if
    } // end for
}

// Find line: Estimate the position of the robot w.r.t the line and calculate the value to use for PD Control
int findLine()
{
    // from pololu read_line
    long avg = 0; // this is for the weighted total, which is long before division
    int sum = 0; // this is for the denominator which is <= 64000
    flagOnLine = false; // false: all sensors see font
                        // true: at least one sensor sees line

    checkOnLine(); // Function call

    for (unsigned int i = 0; i < NUM_SENSORS; i++) {
        int value = sensorColour[i];
#if (DEBUG_QTR_1)
        Serial.print(value);
        Serial.print("\t");
#endif
#if (DEBUG_QTR_2)
        Serial.print(online[i]);
        Serial.print('\t');
#endif
        //!!!!ATTENTION!!!!!
        // The expected average (for QTR-8RC) would be 0-7000 , with reference point = 3500
        if (value > 30) { // only average in values that are above a noise threshold
            avg += (long)(value) * (i * 1000);
            sum += value;
        }
    }

#ifdef DEBUG
    Serial.println(); // Uncomment when any one of the print lines above is uncommented to add a new line on each loop
#endif

    //************************* CHECK LINE VISIBILITY ****************************//

    // ************ LINE NOT AVAILABLE ************
    if (!flagOnLine) { // No sensor sees line, all sensors see font

        // RIGHT ANGLE
        if (flagAngleDetection == 1) {
            // Serial.println("right angle"); // Used for debug, leave commented during run
            return 1000; // special case (no line available)
        }
        // LEFT ANGLE
        else if (flagAngleDetection == -1) {
            // Serial.println("right angle"); // Used for debug, leave commented during run
            return -1000; // special case (no line available)
        }
    }

    // ************ LINE AVAILABLE ************
    else { // Line is available

        //*************SENSOR'S HISTORY**************//

        // check if the right/left sensor is on the line
        // needed to distinguish right angle from dashed line
        if (online[0]) {
            flagAngleDetection = -1;
        }
        else if (online[NUM_SENSORS - 1]) {
            flagAngleDetection = 1;
        }
        else {
            flagAngleDetection = 0;
        }
    }

    // ************* REGULAR LINE *************
    lastValue = avg / sum;
    // Value is calculated from 0 to 7000, 3500 means the line is in the middle
    // Derive it by ten to make it from 0 to 700 and substract 350 so now when the value is 0
    // means the line is in the middle
    return lastValue / 10 - 350; // normalise to [-350,+350]
                                 // end line is visible
}

// PD Control: Run the PD controller and return the result.
int control()
{
    pidError = PID_REF + reflectanceOutput; // Error is calculated as the distance of the sensor board's center from the line.

    pidErrorSum += pidError * LOOP_FREQUENCY; // Calculate integral.

    // Calculate derivative (sampling time is considered).
    pidErrorDerivative = 10.0 * (double)(pidError - pidErrorPrev) / (double)LOOP_FREQUENCY;

    // Save the last value of the error (for next iteration).
    pidErrorPrev = pidError;

    return PID_KP * pidError + PID_KI * pidErrorSum + PID_KD * pidErrorDerivative;
}

// Command motors: Calculate the speed and direction of each motor
// v: linear velocity, th: angular velocity
void commandMotors(int v, int th)
{
    pwmLeft = -v - th;
    pwmRight = v - th;

    // Saturate within [-255,+255].
    if (pwmLeft < -MAX_PWM) {
        pwmLeft = -MAX_PWM;
    }
    else if (pwmLeft > MAX_PWM) {
        pwmLeft = MAX_PWM;
    }
    if (pwmRight > MAX_PWM) {
        pwmRight = MAX_PWM;
    }
    else if (pwmRight < -MAX_PWM) {
        pwmRight = -MAX_PWM;
    }

    // The signs of pwmLeft, pwmRight determine the direction of the motors
    // If a motor turns to the wrong direction simply change operator from >= to <= and vice versa
    if (pwmLeft <= 0) {
        digitalWrite(pin_MOTOR_DRIVER_LEFT_IN_1, HIGH);
        digitalWrite(pin_MOTOR_DRIVER_LEFT_IN_2, LOW);
    }
    else {
        digitalWrite(pin_MOTOR_DRIVER_LEFT_IN_1, LOW);
        digitalWrite(pin_MOTOR_DRIVER_LEFT_IN_2, HIGH);
    }
    if (pwmRight <= 0) {
        digitalWrite(pin_MOTOR_DRIVER_RIGHT_IN_1, LOW);
        digitalWrite(pin_MOTOR_DRIVER_RIGHT_IN_2, HIGH);
    }
    else {
        digitalWrite(pin_MOTOR_DRIVER_RIGHT_IN_1, HIGH);
        digitalWrite(pin_MOTOR_DRIVER_RIGHT_IN_2, LOW);
    }
    // Write the values to the motors
    analogWrite(pin_MOTOR_DRIVER_LEFT_PWM, abs(pwmLeft));
    analogWrite(pin_MOTOR_DRIVER_RIGHT_PWM, abs(pwmRight));
}

// Process: Function that processes the result of findLine and calls the appropriate function
void process(int reflectanceOutput)
{
    if (reflectanceOutput <= 350 && reflectanceOutput >= -350) // Robot on-line.
    {
        linearVel = PWM_CENTERED;
        angularVel = control();
    }
    else // Robot lost line.
    {
        linearVel = 0;
        if (reflectanceOutput == 1000) {
            // Hard Turn Right.
            // Serial.println("Handling right angle..."); // Used for debugging, leave commented during run.
            angularVel = PWM_90DEG_ANGLE;
        }
        else {
            // Hard Turn Left.
            // Serial.println("Handling left angle..."); // Used for debugging, leave commented during run.
            angularVel = -PWM_90DEG_ANGLE;
        }
    }
}

//----------------------- SETUP FUNCTION ----------------------------//
int main()
{
#if (DEBUG)
    Serial.begin(115200);
    BT.begin(115200);
#endif
    // Motor Driver
    pinMode(pin_MOTOR_DRIVER_LEFT_PWM, OUTPUT);
    pinMode(pin_MOTOR_DRIVER_RIGHT_PWM, OUTPUT);
    pinMode(pin_MOTOR_DRIVER_LEFT_IN_1, OUTPUT);
    pinMode(pin_MOTOR_DRIVER_LEFT_IN_2, OUTPUT);
    pinMode(pin_MOTOR_DRIVER_RIGHT_IN_1, OUTPUT);
    pinMode(pin_MOTOR_DRIVER_RIGHT_IN_2, OUTPUT);
    pinMode(pin_MOTOR_DRIVER_STBY, OUTPUT);

    // LEDs
    pinMode(pin_LED_RED, OUTPUT);
    pinMode(pin_LED_BLUE, OUTPUT);
    pinMode(pin_LED_GREEN, OUTPUT);

    // Buzzer
    pinMode(pin_BUZZER, OUTPUT);

    // Push Button
    pinMode(pin_PUSH_BTN, INPUT);

    digitalWrite(pin_MOTOR_DRIVER_STBY, HIGH); // Motor Driver State - Active
    delay(1500); // Wait before beginning calibration.

    digitalWrite(pin_LED_BLUE, HIGH); // Start Calibration.

    for (int i = 0; i < 500; i++) // make the calibration take about 10 seconds
    {
        qtrrc.calibrate(); // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }
    digitalWrite(pin_LED_BLUE, LOW); // calibration over

    // Wait until the pushbutton is pushed to start the robot.
    while (1) {
        if (digitalRead(pin_PUSH_BTN) == HIGH) {
            break;
        }
    }
    // Detect Color of line on starting point
    readSensors();

    //------------------------- MAIN LOOP -------------------------------//
    while (1) {
        /// Get current time.
        currentTime = millis();
        // Accurate timekeeping: Makes the sure the code doesn't run in less than LOOP_FREQUENCY ms.
        if (currentTime - previousTime > LOOP_FREQUENCY) {
            // Save the last time we entered the loop.
            previousTime = currentTime;

            // Read data from all sensors.
            readSensors();

            // Estimate robot position in reference to the line.
            reflectanceOutput = findLine();
#if (DEBUG_REFLECT_OUTPUT)
            Serial.println(reflectanceOutput); // Used for debugging, leave commented during run.
#endif
            // Calculate Linear and Angular velocity.
            process(reflectanceOutput);

            // Drive the motors with the Appropriate PWM.
            commandMotors(linearVel, angularVel);

        } // Timekeeper
    }
}