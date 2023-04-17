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

//------------------------- BLUETOOTH -------------------------------//

#define BT Serial1

#define DEBUG 0
#define DEBUG_QTR_1 0
#define DEBUG_QTR_2 0
#define DEBUG_REFLECT_OUTPUT 0

//------------------- ALGORITHM COEFFICIENTS ------------------------//

// QTR Sensors Setup.
static const unsigned int NUM_SENSORS = 8U; // Number of sensors used in sensor board.
static const unsigned int TIMEOUT = 1000U; // Waits for 1000 microseconds for sensor outputs to go low, for Digital QTR.
static const unsigned int EMITTER_PIN = QTR_NO_EMITTER_PIN; // Keep emitters ON.
// static const unsigned int NUM_SAMPLES_PER_SENSOR = 4U;      	// Uncomment if using Analog QTR.

// PID Control Parameters.
static const uint8_t PID_REF = 0U;
static const float PID_KP = 0.9;
static const float PID_KI = 0.0;
static const float PID_KD = 0.65;

// Maximum Robot Speed.
static const int MAX_PWM = 155; // Max PWM output to protect the motors.

// Main Loop Frequency.
static const unsigned int LOOP_FREQUENCY = 3U; // Loop Frequency in ms.

// Speed variables.
static const int PWM_CENTERED = 95; // Max PWM for linear velocity when line is centered.

static const int PWM_90DEG_ANGLE = 170; // Angular velocity used in right or left 90 deg angle.

//-----------------------PIN DEFINITIONS-----------------------------//

// IR Sensors.
static const uint8_t pin_QTR_1 = 21U;
static const uint8_t pin_QTR_2 = 20U;
static const uint8_t pin_QTR_3 = 19U;
static const uint8_t pin_QTR_4 = 18U;
static const uint8_t pin_QTR_5 = 17U;
static const uint8_t pin_QTR_6 = 16U;
static const uint8_t pin_QTR_7 = 15U;
static const uint8_t pin_QTR_8 = 14U;

// Motor Driver.
static const uint8_t pin_MOTOR_DRIVER_LEFT_PWM = 3U;
static const uint8_t pin_MOTOR_DRIVER_RIGHT_PWM = 4U;
static const uint8_t pin_MOTOR_DRIVER_LEFT_IN_1 = 6U;
static const uint8_t pin_MOTOR_DRIVER_LEFT_IN_2 = 5U;
static const uint8_t pin_MOTOR_DRIVER_STBY = 2U;
static const uint8_t pin_MOTOR_DRIVER_RIGHT_IN_1 = 7U;
static const uint8_t pin_MOTOR_DRIVER_RIGHT_IN_2 = 8U;

// LEDs.
static const uint8_t pin_LED_RED = 9U;
static const uint8_t pin_LED_GREEN = 11U;
static const uint8_t pin_LED_BLUE = 10U;

// Push Button.
static const uint8_t pin_PUSH_BTN = 12U; // Normally LOW

// Buzzer.
static const uint8_t pin_BUZZER = 22U;

// Available General Purpose Pins
// 13
// 23
// A14