#if 0
/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
int us2count(int us) {
  return (us * 4096L) / 20000L;
}

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  900 //150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2100 // 600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  //pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  // Drive each servo one at a time using setPWM()
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, us2count(pulselen));
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, us2count(pulselen));
  }

  delay(500);

  // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
  // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior. 
  //for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
  //  pwm.writeMicroseconds(servonum, microsec);
  //}

  //delay(500);
  //for (uint16_t microsec = USMAX; microsec > USMIN; microsec--) {
  //  pwm.writeMicroseconds(servonum, microsec);
  //}

  //delay(500);

  //servonum++;
  //if (servonum > 7) servonum = 0; // Testing the first 8 servo channels
}
#endif
#if 1
/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
int centerPulseLength = 1500;
int minPulseLength = centerPulseLength - 600;
int maxPulseLength = centerPulseLength + 600;


uint8_t servoFirst = 13;
uint8_t servoLast = 14;

#define HEAD_PAN_SERVO 0
#define HEAD_PAN_PW_MIN 900
#define HEAD_PAN_ADC_MIN 0
#define HEAD_PAN_ANGLE_MIN -45
#define HEAD_PAN_PW_CENTER 1500
#define HEAD_PAN_ADC_CENTER 512
#define HEAD_PAN_ANGLE_CENTER 0
#define HEAD_PAN_PW_MAX 2100
#define HEAD_PAN_ADC_MAX 1024
#define HEAD_PAN_ANGLE_MAX 45
#define HEAD_PAN_VECTOR_LENGTH 400
#define HEAD_PAN_PW_FILTER_COEFFICIENT 0.1
#define HEAD_PAN_ADC_FILTER_COEFFICIENT 0.5

#define HEAD_TILT_SERVO 1
#define HEAD_TILT_PW_MIN 900
#define HEAD_TILT_ADC_MIN 0
#define HEAD_TILT_ANGLE_MIN -45
#define HEAD_TILT_PW_CENTER 1500
#define HEAD_TILT_ADC_CENTER 512
#define HEAD_TILT_ANGLE_CENTER 512
#define HEAD_TILT_PW_MAX 2100
#define HEAD_TILT_ADC_MAX 1024
#define HEAD_TILT_ANGLE_MAX 45
#define HEAD_TILT_VECTOR_LENGTH 300
#define HEAD_TILT_PW_FILTER_COEFFICIENT 0.1
#define HEAD_TILT_ADC_FILTER_COEFFICIENT 0.5

#define LEFT_EYE_PAN_SERVO 3
#define LEFT_EYE_PAN_PW_MIN 900
#define LEFT_EYE_PAN_ANGLE_MIN -45
#define LEFT_EYE_PAN_ADC_MIN 0
#define LEFT_EYE_PAN_PW_CENTER 1500
#define LEFT_EYE_PAN_ANGLE_CENTER 0
#define LEFT_EYE_PAN_ADC_CENTER 0
#define LEFT_EYE_PAN_PW_MAX 2100
#define LEFT_EYE_PAN_ANGLE_MAX 45
#define LEFT_EYE_PAN_ADC_MAX 1024
#define LEFT_EYE_PAN_PW_FILTER_COEFFICIENT 0.2
#define LEFT_EYE_PAN_ADC_FILTER_COEFFICIENT 0.5

#define LEFT_EYE_TILT_SERVO 4
#define LEFT_EYE_TILT_PW_MIN 900
#define LEFT_EYE_TILT_ANGLE_MIN -45
#define LEFT_EYE_TILT_ADC_MIN 0
#define LEFT_EYE_TILT_PW_CENTER 1500
#define LEFT_EYE_TILT_ANGLE_CENTER 0
#define LEFT_EYE_TILT_ADC_CENTER 0
#define LEFT_EYE_TILT_PW_MAX 2100
#define LEFT_EYE_TILT_ANGLE_MAX 45
#define LEFT_EYE_TILT_ADC_MAX 1024
#define LEFT_EYE_TILT_PW_FILTER_COEFFICIENT 0.2
#define LEFT_EYE_TILT_ADC_FILTER_COEFFICIENT 0.5

#define RIGHT_EYE_PAN_SERVO 5
#define RIGHT_EYE_PAN_PW_MIN 900
#define RIGHT_EYE_PAN_ANGLE_MIN -45
#define RIGHT_EYE_PAN_ADC_MIN 0
#define RIGHT_EYE_PAN_PW_CENTER 1500
#define RIGHT_EYE_PAN_ANGLE_CENTER 0
#define RIGHT_EYE_PAN_ADC_CENTER 0
#define RIGHT_EYE_PAN_PW_MAX 2100
#define RIGHT_EYE_PAN_ANGLE_MAX 45
#define RIGHT_EYE_PAN_ADC_MAX 1024
#define RIGHT_EYE_PAN_PW_FILTER_COEFFICIENT 0.2
#define RIGHT_EYE_PAN_ADC_FILTER_COEFFICIENT 0.5

#define RIGHT_EYE_TILT_SERVO 6
#define RIGHT_EYE_TILT_PW_MIN 900
#define RIGHT_EYE_TILT_ANGLE_MIN -45
#define RIGHT_EYE_TILT_ADC_MIN 0
#define RIGHT_EYE_TILT_PW_CENTER 1500
#define RIGHT_EYE_TILT_ANGLE_CENTER 0
#define RIGHT_EYE_TILT_ADC_CENTER 0
#define RIGHT_EYE_TILT_PW_MAX 2100
#define RIGHT_EYE_TILT_ANGLE_MAX 45
#define RIGHT_EYE_TILT_ADC_MAX 1024
#define RIGHT_EYE_TILT_PW_FILTER_COEFFICIENT 0.2
#define RIGHT_EYE_TILT_ADC_FILTER_COEFFICIENT 0.5

#define MOUTH_SERVO 2
#define MOUTH_CENTER_PW 1500
#define MOUTH_PW_MIN 900
#define MOUTH_PW_MAX 2100
#define MOUTH_PW_MIN 900
#define MOUTH_ADC_MIN 0
#define MOUTH_ANGLE_MIN -45
#define MOUTH_PW_CENTER 1500
#define MOUTH_ADC_CENTER 512
#define MOUTH_ANGLE_CENTER 512
#define MOUTH_PW_MAX 2100
#define MOUTH_ADC_MAX 1024
#define MOUTH_ANGLE_MAX 45
#define MOUTH_PW_FILTER_COEFFICIENT 0.1
#define MOUTH_ADC_FILTER_COEFFICIENT 0.5

#define HEAD_PAN_VECTOR_LENGTH 600
#define HEAD_TILT_VECTOR_LENGTH 350
#define EYE_PAN_VECTOR_LENGTH 600
#define EYE_TILT_VECTOR_LENGTH 350
#define MOUTH_VECTOR_LENGTH 300

static int centerPulseWidth = 1500;
static int minPulseWidth = centerPulseWidth - 600;
static int maxPulseWidth = centerPulseWidth + 600;

int us2count(int us) {
  return (us * 4096L) / 20000L;
}


struct Servo {

  float minPw;
  float minAngle;
  float minAdc;
  float centerPw;
  float centerAngle;
  float centerAdc;
  float maxPw;
  float maxAngle;
  float maxAdc;
  float targetPw;
  float filteredPw;
  float filteredAdc;
  float pwFilterCoefficient;
  float adcFilterCoefficient;

  float setTarget(float targetPw) { targetPw = targetPw; }

  void updatePwFilter() {
    filteredPw = filteredPw + pwFilterCoefficient*(targetPw - filteredPw);
  }

  float getPwAngle() {
    return ((maxAngle - minAngle) * (maxPw - filteredPw) / (maxPulseWidth - minPulseWidth)) - 45;
  }

  void updateAdcFilter(float adc) {
    filteredAdc = filteredAdc + adcFilterCoefficient*(adc - filteredAdc);
    if (filteredAdc > maxAdc) maxAdc = filteredAdc;
    if (filteredAdc < minAdc) minAdc = filteredAdc;
  }

  float getAdcAngle() {
    return ((maxAngle - minAngle) * (filteredAdc - minAdc) / (maxAdc - minAdc)) - 45;
  }

};

Servo headPanServo = {
  .minPw = HEAD_PAN_PW_MIN,
  .minAngle = HEAD_PAN_ANGLE_MIN,
  .minAdc = HEAD_PAN_ADC_MAX,
  .centerPw = HEAD_PAN_PW_CENTER,
  .centerAngle = HEAD_PAN_ANGLE_CENTER,
  .centerAdc = HEAD_PAN_ADC_CENTER,
  .maxPw = HEAD_PAN_PW_MAX,
  .maxAngle = HEAD_PAN_ANGLE_MAX,
  .maxAdc = HEAD_PAN_ADC_MIN,
  .targetPw = HEAD_PAN_PW_CENTER,
  .filteredPw = HEAD_PAN_PW_CENTER,
  .filteredAdc = HEAD_PAN_ADC_CENTER,
  .pwFilterCoefficient = HEAD_PAN_PW_FILTER_COEFFICIENT,
  .adcFilterCoefficient = HEAD_PAN_ADC_FILTER_COEFFICIENT
};

Servo headTiltServo = {
  .minPw = HEAD_TILT_PW_MIN,
  .minAngle = HEAD_TILT_ANGLE_MIN,
  .minAdc = HEAD_TILT_ADC_MAX,
  .centerPw = HEAD_TILT_PW_CENTER,
  .centerAngle = HEAD_TILT_ANGLE_CENTER,
  .centerAdc = HEAD_TILT_ADC_CENTER,
  .maxPw = HEAD_TILT_PW_MAX,
  .maxAngle = HEAD_TILT_ANGLE_MAX,
  .maxAdc = HEAD_TILT_ADC_MIN,
  .targetPw = HEAD_TILT_PW_CENTER,
  .filteredPw = HEAD_TILT_PW_CENTER,
  .filteredAdc = HEAD_TILT_ADC_CENTER,
  .pwFilterCoefficient = HEAD_TILT_PW_FILTER_COEFFICIENT,
  .adcFilterCoefficient = HEAD_TILT_ADC_FILTER_COEFFICIENT
};

Servo leftEyePanServo = {
  .minPw = LEFT_EYE_PAN_PW_MIN,
  .minAngle = LEFT_EYE_PAN_ANGLE_MIN,
  .minAdc = LEFT_EYE_PAN_ADC_MAX,
  .centerPw = LEFT_EYE_PAN_PW_CENTER,
  .centerAngle = LEFT_EYE_PAN_ANGLE_CENTER,
  .centerAdc = LEFT_EYE_PAN_ADC_CENTER,
  .maxPw = LEFT_EYE_PAN_PW_MAX,
  .maxAngle = LEFT_EYE_PAN_ANGLE_MAX,
  .maxAdc = LEFT_EYE_PAN_ADC_MIN,
  .targetPw = LEFT_EYE_PAN_PW_CENTER,
  .filteredPw = LEFT_EYE_PAN_PW_CENTER,
  .filteredAdc = LEFT_EYE_PAN_ADC_CENTER,
  .pwFilterCoefficient = LEFT_EYE_PAN_PW_FILTER_COEFFICIENT,
  .adcFilterCoefficient = LEFT_EYE_PAN_ADC_FILTER_COEFFICIENT
};

Servo leftEyeTiltServo = {
  .minPw = LEFT_EYE_TILT_PW_MIN,
  .minAngle = LEFT_EYE_TILT_ANGLE_MIN,
  .minAdc = LEFT_EYE_TILT_ADC_MAX,
  .centerPw = LEFT_EYE_TILT_PW_CENTER,
  .centerAngle = LEFT_EYE_TILT_ANGLE_CENTER,
  .centerAdc = LEFT_EYE_TILT_ADC_CENTER,
  .maxPw = LEFT_EYE_TILT_PW_MAX,
  .maxAngle = LEFT_EYE_TILT_ANGLE_MAX,
  .maxAdc = LEFT_EYE_TILT_ADC_MIN,
  .targetPw = RIGHT_EYE_TILT_PW_CENTER,
  .filteredPw = LEFT_EYE_TILT_PW_CENTER,
  .filteredAdc = LEFT_EYE_TILT_ADC_CENTER,
  .pwFilterCoefficient = LEFT_EYE_TILT_PW_FILTER_COEFFICIENT,
  .adcFilterCoefficient = LEFT_EYE_TILT_ADC_FILTER_COEFFICIENT
};

Servo rightEyePanServo = {
  .minPw = RIGHT_EYE_PAN_PW_MIN,
  .minAngle = RIGHT_EYE_PAN_ANGLE_MIN,
  .minAdc = RIGHT_EYE_PAN_ADC_MAX,
  .centerPw = RIGHT_EYE_PAN_PW_CENTER,
  .centerAngle = RIGHT_EYE_PAN_ANGLE_CENTER,
  .centerAdc = RIGHT_EYE_PAN_ADC_CENTER,
  .maxPw = RIGHT_EYE_PAN_PW_MAX,
  .maxAngle = RIGHT_EYE_PAN_ANGLE_MAX,
  .maxAdc = RIGHT_EYE_PAN_ADC_MIN,
  .targetPw = RIGHT_EYE_PAN_PW_CENTER,
  .filteredPw = RIGHT_EYE_PAN_PW_CENTER,
  .filteredAdc = RIGHT_EYE_PAN_ADC_CENTER,
  .pwFilterCoefficient = RIGHT_EYE_PAN_PW_FILTER_COEFFICIENT,
  .adcFilterCoefficient = RIGHT_EYE_PAN_ADC_FILTER_COEFFICIENT
};

Servo rightEyeTiltServo = {
  .minPw = RIGHT_EYE_TILT_PW_MIN,
  .minAngle = RIGHT_EYE_TILT_ANGLE_MIN,
  .minAdc = RIGHT_EYE_TILT_ADC_MAX,
  .centerPw = RIGHT_EYE_TILT_PW_CENTER,
  .centerAngle = RIGHT_EYE_TILT_ANGLE_CENTER,
  .centerAdc = RIGHT_EYE_TILT_ADC_CENTER,
  .maxPw = RIGHT_EYE_TILT_PW_MAX,
  .maxAngle = RIGHT_EYE_TILT_ANGLE_MAX,
  .maxAdc = RIGHT_EYE_TILT_ADC_MIN,
  .targetPw = RIGHT_EYE_TILT_PW_CENTER,
  .filteredPw = RIGHT_EYE_TILT_PW_CENTER,
  .filteredAdc = RIGHT_EYE_TILT_ADC_CENTER,
  .pwFilterCoefficient = RIGHT_EYE_TILT_PW_FILTER_COEFFICIENT,
  .adcFilterCoefficient = RIGHT_EYE_TILT_ADC_FILTER_COEFFICIENT
};

Servo mouthServo = {
  .minPw = MOUTH_PW_MIN,
  .minAngle = MOUTH_ANGLE_MIN,
  .minAdc = MOUTH_ADC_MAX,
  .centerPw = MOUTH_PW_CENTER,
  .centerAngle = MOUTH_ANGLE_CENTER,
  .centerAdc = MOUTH_ADC_CENTER,
  .maxPw = MOUTH_PW_MAX,
  .maxAngle = MOUTH_ANGLE_MAX,
  .maxAdc = MOUTH_ADC_MIN,
  .targetPw = MOUTH_PW_CENTER,
  .filteredPw = MOUTH_PW_CENTER,
  .filteredAdc = MOUTH_ADC_CENTER,
  .pwFilterCoefficient = MOUTH_PW_FILTER_COEFFICIENT,
  .adcFilterCoefficient = MOUTH_ADC_FILTER_COEFFICIENT
};

float headAngle = 0;
float eyeAngle = 0;

float updateFilter(float current, float sample, float coefficient) {
  return current + coefficient*(sample - current);
}

int pirInputPin = 10;
int pirState = LOW;
int runAnimation = false;
int trigPin = 9;
int echoPin = 8;
float duration_us = 0.0;
float distance_cm = 0.0;
float max_distance = 400.0;

void updateDistanceFilter(float distance) {
  max_distance = max_distance + .001*(distance - max_distance);
}

void updatePosition() {

    headPanServo.updatePwFilter();
    headPanServo.updateAdcFilter(analogRead(0));

    headTiltServo.updatePwFilter();
    headTiltServo.updateAdcFilter(analogRead(1));

    leftEyePanServo.updatePwFilter();
    leftEyeTiltServo.updatePwFilter();

    rightEyePanServo.updatePwFilter();
    rightEyeTiltServo.updatePwFilter();

    mouthServo.updatePwFilter();

    uint16_t headPanCount;
    uint16_t headTiltCount;
    uint16_t leftEyePanCount;
    uint16_t leftEyeTiltCount;
    uint16_t rightEyePanCount;
    uint16_t rightEyeTiltCount;
    uint16_t mouthCount;

    headPanCount = us2count(headPanServo.filteredPw);
    pwm.setPWM(HEAD_PAN_SERVO, 0, headPanCount);

    headTiltCount = us2count(headTiltServo.filteredPw);
    pwm.setPWM(HEAD_TILT_SERVO, 0, headTiltCount);

    leftEyePanCount = us2count(leftEyePanServo.filteredPw);
    pwm.setPWM(LEFT_EYE_PAN_SERVO, 0, leftEyePanCount);

    leftEyeTiltCount = us2count(leftEyeTiltServo.filteredPw);
    pwm.setPWM(LEFT_EYE_TILT_SERVO, 0, leftEyeTiltCount);

    rightEyePanCount = us2count(rightEyePanServo.filteredPw);
    pwm.setPWM(RIGHT_EYE_PAN_SERVO, 0, rightEyePanCount);

    rightEyeTiltCount = us2count(rightEyeTiltServo.filteredPw);
    pwm.setPWM(RIGHT_EYE_TILT_SERVO, 0, rightEyeTiltCount);

    mouthCount = us2count(mouthServo.filteredPw);
    pwm.setPWM(MOUTH_SERVO, 0, mouthCount);

    if (Serial) {
      Serial.print("PIR=");
      Serial.print(pirState);
      Serial.print(" anim=");
      Serial.print(runAnimation);
      //Serial.print(" max_distance=");
      //Serial.print(max_distance);
      Serial.print(" distance=");
      Serial.print(distance_cm);
      Serial.print(" effect=");
      Serial.print(int(distance_cm/50));
      //Serial.print(" head pan pw=");
      //Serial.print(headPanServo.filteredPw);
      //Serial.print(" count=");
      //Serial.print(headPanCount);
      //Serial.print(" adc=");
      //Serial.print(headPanServo.filteredAdc);
      //Serial.print(" pw-angle=");
      //Serial.print(headPanServo.getPwAngle());
      //Serial.print(" adc-angle=");
      //Serial.print(headPanServo.getAdcAngle());
      //Serial.print(" head tilt pw=");
      //Serial.print(headTiltServo.filteredPw);
      //Serial.print(" count=");
      //Serial.print(headTiltCount);
      //Serial.print(" pw-angle=");
      //Serial.print(headTiltServo.getPwAngle());
      //Serial.print(" adc=");
      //Serial.print(headTiltServo.filteredAdc);
      //Serial.print(" adc-angle=");
      //Serial.print(headTiltServo.getAdcAngle());

      Serial.print(" l-eye pan pw=");
      Serial.print(leftEyePanServo.filteredPw);
      //Serial.print(" count=");
      //Serial.print(leftEyePanCount);
      Serial.print(" l-eye tilt pw=");
      Serial.print(leftEyeTiltServo.filteredPw);
      //Serial.print(" count=");
      //Serial.print(leftEyeTiltCount);
      Serial.print(" r-eye pan pw=");
      Serial.print(rightEyePanServo.filteredPw);
      //Serial.print(" count=");
      //Serial.print(rightEyePanCount);
      Serial.print(" r-eye tilt pw=");
      Serial.print(rightEyeTiltServo.filteredPw);
      //Serial.print(" count=");
      //Serial.print(rightEyeTiltCount);
      Serial.print(" mouth pw=");
      Serial.print(mouthServo.filteredPw);
      //Serial.print(" count=");
      //Serial.print(mouthCount);
      Serial.println();
    }

    //delay(1000/50);
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  static double pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 50;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void setServoAngle(int servoNum, int angle) {
  static double scale = (maxPulseLength - minPulseLength) / 180.0;
  angle = angle < 0 ? 0 : angle > 180 ? 180 : angle;
  pwm.setPWM(servoNum, 0, minPulseLength + (scale * angle));
}

void saveServoStates() {
  int address = 0;
  int stateSaved = 1234;
  EEPROM.put(address,stateSaved);
  address += sizeof(stateSaved);
  int step = sizeof(Servo);
  EEPROM.put(address,headPanServo);
  address += step;
  EEPROM.put(address,headTiltServo);
  address += step;
  EEPROM.put(address,leftEyePanServo);
  address += step;
  EEPROM.put(address,leftEyeTiltServo);
  address += step;
  EEPROM.put(address,rightEyePanServo);
  address += step;
  EEPROM.put(address,rightEyeTiltServo);
  address += step;
  EEPROM.put(address,mouthServo);
}

void restoreServoStates() {
  int address = 0;
  int stateSaved;
  EEPROM.get(address,stateSaved);
  if (stateSaved != 1234) return;
  address += sizeof(stateSaved);
  int step = sizeof(Servo);
  EEPROM.get(address,headPanServo);
  address += step;
  EEPROM.get(address,headTiltServo);
  address += step;
  EEPROM.get(address,leftEyePanServo);
  address += step;
  EEPROM.get(address,leftEyeTiltServo);
  address += step;
  EEPROM.get(address,rightEyePanServo);
  address += step;
  EEPROM.get(address,rightEyeTiltServo);
  address += step;
  EEPROM.get(address,mouthServo);
}

void setup() {

  pinMode(pirInputPin,INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);

  Serial.begin(115200);
  //while(Serial.read() == -1);

  restoreServoStates();

  
#ifdef ESP8266
  Wire.pins(2, 14);   // ESP8266 can use any two pins, such as SDA to #2 and SCL to #14
#endif

  pwm.begin();
  
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates

  yield();
}

int animationIndex = 0;

void loop() {

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(echoPin, HIGH);
  distance_cm = 0.017 * duration_us;
  //updateDistanceFilter(distance_cm);

  int inputValue = digitalRead(pirInputPin);  // read input value
  
  if (inputValue == HIGH) {
    if (pirState == LOW) {
      pirState = HIGH;
      runAnimation = true;
      animationIndex = random(0,5);
    }
  } else {
    if (pirState == HIGH){
      pirState = LOW;
      runAnimation = false;
      headPanServo.targetPw = headPanServo.centerPw;
      headTiltServo.targetPw = headTiltServo.centerPw;
      leftEyePanServo.targetPw = leftEyePanServo.centerPw;
      leftEyeTiltServo.targetPw = leftEyeTiltServo.centerPw;
      rightEyePanServo.targetPw = rightEyePanServo.centerPw;
      rightEyeTiltServo.targetPw = rightEyeTiltServo.centerPw;
      mouthServo.targetPw = mouthServo.centerPw;
    }
  }

  if (runAnimation) {
    switch (animationIndex) {
      case 0:
        headTiltServo.targetPw = HEAD_TILT_PW_MIN;
        leftEyeTiltServo.targetPw = LEFT_EYE_TILT_PW_MIN;
        rightEyeTiltServo.targetPw = RIGHT_EYE_TILT_PW_MAX;
        mouthServo.targetPw = MOUTH_PW_MAX;
        break;
      case 1:
        headPanServo.targetPw = HEAD_PAN_PW_MIN;
        leftEyePanServo.targetPw = LEFT_EYE_PAN_PW_MIN;
        rightEyePanServo.targetPw = RIGHT_EYE_PAN_PW_MIN;
        mouthServo.targetPw = MOUTH_PW_MAX;
        break;
      case 2:
        headTiltServo.targetPw = HEAD_TILT_PW_MAX;
        leftEyeTiltServo.targetPw = LEFT_EYE_TILT_PW_MAX;
        rightEyeTiltServo.targetPw = RIGHT_EYE_TILT_PW_MIN;
        mouthServo.targetPw = MOUTH_PW_MIN;
        break;
      case 3:
        headPanServo.targetPw = HEAD_PAN_PW_MAX;
        leftEyePanServo.targetPw = LEFT_EYE_PAN_PW_MAX;
        rightEyePanServo.targetPw = RIGHT_EYE_PAN_PW_MAX;
        mouthServo.targetPw = MOUTH_PW_MIN;
        break;
      case 4:
        leftEyePanServo.targetPw = random(LEFT_EYE_PAN_PW_MIN,LEFT_EYE_PAN_PW_MAX);
        leftEyeTiltServo.targetPw = random(LEFT_EYE_TILT_PW_MIN,LEFT_EYE_TILT_PW_MAX);
        rightEyePanServo.targetPw = random(RIGHT_EYE_PAN_PW_MIN,RIGHT_EYE_PAN_PW_MAX);
        rightEyeTiltServo.targetPw = random(RIGHT_EYE_TILT_PW_MIN,RIGHT_EYE_TILT_PW_MAX);
      case 5:
        eyeAngle += 5;
        leftEyePanServo.targetPw = leftEyePanServo.centerPw + EYE_PAN_VECTOR_LENGTH * cos(eyeAngle * PI/180.0);
        leftEyeTiltServo.targetPw = leftEyeTiltServo.centerPw - EYE_TILT_VECTOR_LENGTH * sin(eyeAngle * PI/180.0);
        rightEyePanServo.targetPw = rightEyePanServo.centerPw + EYE_PAN_VECTOR_LENGTH * cos(eyeAngle * PI/180.0);
        rightEyeTiltServo.targetPw = rightEyeTiltServo.centerPw + EYE_TILT_VECTOR_LENGTH * sin(eyeAngle * PI/180.0);

    }
  }

  if (Serial) {
    int c =  Serial.read();
    if (c != -1) runAnimation = false;
    switch (c) {
      case -1:
        break;

      case 'K':
        headTiltServo.targetPw+=20;
        if (headTiltServo.targetPw > HEAD_TILT_PW_MAX) headTiltServo.targetPw = HEAD_TILT_PW_MAX;
        break;
      case 'k':
        headTiltServo.targetPw+=1;
        if (headTiltServo.targetPw > HEAD_TILT_PW_MAX) headTiltServo.targetPw = HEAD_TILT_PW_MAX;
        break;
      case 'I':
        headTiltServo.targetPw-=20;
        if (headTiltServo.targetPw < HEAD_TILT_PW_MIN) headTiltServo.targetPw = HEAD_TILT_PW_MIN;
        break;
      case 'i':
        headTiltServo.targetPw-=1;
        if (headTiltServo.targetPw < HEAD_TILT_PW_MIN) headTiltServo.targetPw = HEAD_TILT_PW_MIN;
        break;

      case 'L':
        headPanServo.targetPw+=20;
        if (headPanServo.targetPw > HEAD_PAN_PW_MAX) headPanServo.targetPw = HEAD_PAN_PW_MAX;
        break;
      case 'l':
        headPanServo.targetPw+=1;
        if (headPanServo.targetPw > HEAD_PAN_PW_MAX) headPanServo.targetPw = HEAD_PAN_PW_MAX;
        break;
      case 'J':
        headPanServo.targetPw-=20;
        if (headPanServo.targetPw < HEAD_PAN_PW_MIN) headPanServo.targetPw = HEAD_PAN_PW_MIN;
        break;
      case 'j':
        headPanServo.targetPw-=1;
        if (headPanServo.targetPw < HEAD_PAN_PW_MIN) headPanServo.targetPw = HEAD_PAN_PW_MIN;
        break;

      case 'r':
        headPanServo.targetPw = random(headPanServo.minPw,headPanServo.maxPw);
        headTiltServo.targetPw = random(headTiltServo.minPw,headTiltServo.maxPw);
        leftEyePanServo.targetPw = random(leftEyePanServo.minPw,leftEyePanServo.maxPw);
        leftEyeTiltServo.targetPw = random(leftEyeTiltServo.minPw,leftEyeTiltServo.maxPw);
        rightEyePanServo.targetPw = leftEyePanServo.targetPw;
        rightEyeTiltServo.targetPw = leftEyeTiltServo.targetPw;
        mouthServo.targetPw = mouthServo.maxPw - (mouthServo.maxPw - mouthServo.minPw) * (headTiltServo.targetPw - headTiltServo.minPw) / (headTiltServo.maxPw - headTiltServo.minPw);
        break;

      case 'e':
        leftEyePanServo.targetPw = random(LEFT_EYE_PAN_PW_MIN,LEFT_EYE_PAN_PW_MAX);
        leftEyeTiltServo.targetPw = random(LEFT_EYE_TILT_PW_MIN,LEFT_EYE_TILT_PW_MAX);
        rightEyePanServo.targetPw = random(RIGHT_EYE_PAN_PW_MIN,RIGHT_EYE_PAN_PW_MAX);
        rightEyeTiltServo.targetPw = random(RIGHT_EYE_TILT_PW_MIN,RIGHT_EYE_TILT_PW_MAX);
        break;

      case 'R': {
        headAngle += 5;
        float cosine = cos(headAngle * PI/180.0);
        float sine = sin(headAngle * PI/180.0);
        headPanServo.targetPw = headPanServo.centerPw + HEAD_PAN_VECTOR_LENGTH * cosine;
        headTiltServo.targetPw = headTiltServo.centerPw + HEAD_TILT_VECTOR_LENGTH * sine;
        leftEyePanServo.targetPw = leftEyePanServo.centerPw - EYE_PAN_VECTOR_LENGTH * cosine;
        leftEyeTiltServo.targetPw = leftEyeTiltServo.centerPw + EYE_TILT_VECTOR_LENGTH * sine;
        rightEyePanServo.targetPw = rightEyePanServo.centerPw - EYE_PAN_VECTOR_LENGTH * cosine;
        rightEyeTiltServo.targetPw = rightEyeTiltServo.centerPw - EYE_TILT_VECTOR_LENGTH * sine;
        mouthServo.targetPw = mouthServo.centerPw + HEAD_PAN_VECTOR_LENGTH * sine;
      }
      break;

      case 'E':
        eyeAngle += 5;
        leftEyePanServo.targetPw = leftEyePanServo.centerPw + EYE_PAN_VECTOR_LENGTH * cos(eyeAngle * PI/180.0);
        leftEyeTiltServo.targetPw = leftEyeTiltServo.centerPw - EYE_TILT_VECTOR_LENGTH * sin(eyeAngle * PI/180.0);
        rightEyePanServo.targetPw = rightEyePanServo.centerPw + EYE_PAN_VECTOR_LENGTH * cos(eyeAngle * PI/180.0);
        rightEyeTiltServo.targetPw = rightEyeTiltServo.centerPw + EYE_TILT_VECTOR_LENGTH * sin(eyeAngle * PI/180.0);
        break;

      case '1':
        leftEyePanServo.targetPw += 1;
        if (leftEyePanServo.targetPw > LEFT_EYE_PAN_PW_MAX) leftEyePanServo.targetPw = LEFT_EYE_PAN_PW_MAX;
        break;
      case '!':
        leftEyePanServo.targetPw += 20;
        if (leftEyePanServo.targetPw > LEFT_EYE_PAN_PW_MAX) leftEyePanServo.targetPw = LEFT_EYE_PAN_PW_MAX;
        break;
      case '2':
        leftEyePanServo.targetPw -= 1;
        if (leftEyePanServo.targetPw < LEFT_EYE_PAN_PW_MIN) leftEyePanServo.targetPw = LEFT_EYE_PAN_PW_MIN;
        break;
      case '@':
        leftEyePanServo.targetPw -= 20;
        if (leftEyePanServo.targetPw < LEFT_EYE_PAN_PW_MIN) leftEyePanServo.targetPw = LEFT_EYE_PAN_PW_MIN;
        break;
        
      case '3':
        leftEyeTiltServo.targetPw += 1;
        if (leftEyeTiltServo.targetPw > LEFT_EYE_TILT_PW_MAX) leftEyeTiltServo.targetPw = LEFT_EYE_TILT_PW_MAX;
        break;
      case '#':
        leftEyeTiltServo.targetPw += 20;
        if (leftEyeTiltServo.targetPw > LEFT_EYE_TILT_PW_MAX) leftEyeTiltServo.targetPw = LEFT_EYE_TILT_PW_MAX;
        break;
      case '4':
        leftEyeTiltServo.targetPw -= 1;
        if (leftEyeTiltServo.targetPw < LEFT_EYE_TILT_PW_MIN) leftEyeTiltServo.targetPw = LEFT_EYE_TILT_PW_MIN;
        break;
      case '$':
        leftEyeTiltServo.targetPw -= 20;
        if (leftEyeTiltServo.targetPw < LEFT_EYE_TILT_PW_MIN) leftEyeTiltServo.targetPw = LEFT_EYE_TILT_PW_MIN;
        break;

      case '5':
        rightEyePanServo.targetPw += 1;
        if (rightEyePanServo.targetPw > RIGHT_EYE_PAN_PW_MAX) rightEyePanServo.targetPw = RIGHT_EYE_PAN_PW_MAX;
        break;
      case '%':
        rightEyePanServo.targetPw += 20;
        if (rightEyePanServo.targetPw > RIGHT_EYE_PAN_PW_MAX) rightEyePanServo.targetPw = RIGHT_EYE_PAN_PW_MAX;
        break;
      case '6':
        rightEyePanServo.targetPw -= 1;
        if (rightEyePanServo.targetPw < RIGHT_EYE_PAN_PW_MIN) rightEyePanServo.targetPw = RIGHT_EYE_PAN_PW_MIN;
        break;
      case '^':
        rightEyePanServo.targetPw -= 20;
        if (rightEyePanServo.targetPw < RIGHT_EYE_PAN_PW_MIN) rightEyePanServo.targetPw = RIGHT_EYE_PAN_PW_MIN;
        break;

      case '7':
        rightEyeTiltServo.targetPw -= 1;
        if (rightEyeTiltServo.targetPw < RIGHT_EYE_TILT_PW_MIN) rightEyeTiltServo.targetPw = RIGHT_EYE_TILT_PW_MIN;
        break;
      case '&':
        rightEyeTiltServo.targetPw -= 20;
        if (rightEyeTiltServo.targetPw < RIGHT_EYE_TILT_PW_MIN) rightEyeTiltServo.targetPw = RIGHT_EYE_TILT_PW_MIN;
        break;
      case '8':
        rightEyeTiltServo.targetPw += 1;
        if (rightEyeTiltServo.targetPw > RIGHT_EYE_TILT_PW_MAX) rightEyeTiltServo.targetPw = RIGHT_EYE_TILT_PW_MAX;
        break;
      case '*':
        rightEyeTiltServo.targetPw += 20;
        if (rightEyeTiltServo.targetPw > RIGHT_EYE_TILT_PW_MAX) rightEyeTiltServo.targetPw = RIGHT_EYE_TILT_PW_MAX;
        break;

      case 'c':
        headPanServo.targetPw = headPanServo.centerPw;
        headTiltServo.targetPw = headTiltServo.centerPw;
        leftEyePanServo.targetPw = leftEyePanServo.centerPw;
        leftEyeTiltServo.targetPw = leftEyeTiltServo.centerPw;
        rightEyePanServo.targetPw = rightEyePanServo.centerPw;
        rightEyeTiltServo.targetPw = rightEyeTiltServo.centerPw;
        mouthServo.targetPw = mouthServo.centerPw;
        break;

      case 'C':
        leftEyePanServo.targetPw = leftEyePanServo.centerPw;
        leftEyeTiltServo.targetPw = leftEyeTiltServo.centerPw;
        rightEyePanServo.targetPw = rightEyePanServo.centerPw;
        rightEyeTiltServo.targetPw = rightEyeTiltServo.centerPw;
        break;

      case 's':
        headPanServo.centerPw = headPanServo.targetPw;
        headTiltServo.centerPw = headTiltServo.targetPw;
        saveServoStates();
        break;

     case 'S':
        leftEyePanServo.centerPw = leftEyePanServo.targetPw;
        leftEyeTiltServo.centerPw = leftEyeTiltServo.targetPw;
        rightEyePanServo.centerPw = rightEyePanServo.targetPw;
        rightEyeTiltServo.centerPw = rightEyeTiltServo.targetPw;
        saveServoStates();
        break;

      case ',':
        headTiltServo.targetPw = HEAD_TILT_PW_MIN;
        leftEyeTiltServo.targetPw = LEFT_EYE_TILT_PW_MIN;
        rightEyeTiltServo.targetPw = RIGHT_EYE_TILT_PW_MAX;
        mouthServo.targetPw = MOUTH_PW_MAX;
        break;
      case '<':
        headPanServo.targetPw = HEAD_PAN_PW_MIN;
        leftEyePanServo.targetPw = LEFT_EYE_PAN_PW_MIN;
        rightEyePanServo.targetPw = RIGHT_EYE_PAN_PW_MIN;
        mouthServo.targetPw = MOUTH_PW_MAX;
        break;
      case '.':
        headTiltServo.targetPw = HEAD_TILT_PW_MAX;
        leftEyeTiltServo.targetPw = LEFT_EYE_TILT_PW_MAX;
        rightEyeTiltServo.targetPw = RIGHT_EYE_TILT_PW_MIN;
        mouthServo.targetPw = MOUTH_PW_MIN;
        break;
      case '>':
        headPanServo.targetPw = HEAD_PAN_PW_MAX;
        leftEyePanServo.targetPw = LEFT_EYE_PAN_PW_MAX;
        rightEyePanServo.targetPw = RIGHT_EYE_PAN_PW_MAX;
        mouthServo.targetPw = MOUTH_PW_MIN;
        break;

      case 'm':
        mouthServo.targetPw = random(MOUTH_PW_MIN,MOUTH_PW_MAX);
        break;

      case 'M':
        if (mouthServo.targetPw == MOUTH_PW_MIN) {
          mouthServo.targetPw = MOUTH_PW_MAX;
        }
        else {
          mouthServo.targetPw = MOUTH_PW_MIN;
        }
        break;
  
      case '=':
        leftEyePanServo.pwFilterCoefficient *= 1.1;
        leftEyeTiltServo.pwFilterCoefficient *= 1.1;
        rightEyePanServo.pwFilterCoefficient *= 1.1;
        rightEyeTiltServo.pwFilterCoefficient *= 1.1;
        break;

      case '-':
        leftEyePanServo.pwFilterCoefficient *= .9;
        leftEyeTiltServo.pwFilterCoefficient *= .9;
        rightEyePanServo.pwFilterCoefficient *= .9;
        rightEyeTiltServo.pwFilterCoefficient *= .9;
        break;
    }

    //updatePosition();
  }
  else {
    /*
    eyeAngle += 5;
    leftEyePanServo.targetPw = leftEyePanServo.centerPw + EYE_PAN_VECTOR_LENGTH * cos(eyeAngle * PI/180.0);
    leftEyeTiltServo.targetPw = leftEyeTiltServo.centerPw - EYE_TILT_VECTOR_LENGTH * sin(eyeAngle * PI/180.0);
    rightEyePanServo.targetPw = rightEyePanServo.centerPw + EYE_PAN_VECTOR_LENGTH * cos(eyeAngle * PI/180.0);
    rightEyeTiltServo.targetPw = rightEyeTiltServo.centerPw + EYE_TILT_VECTOR_LENGTH * sin(eyeAngle * PI/180.0);
    */
  }
  updatePosition();
}

#endif