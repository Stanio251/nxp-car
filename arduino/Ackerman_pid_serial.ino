#include <PID_v1.h>
#include <Servo.h>

//rear wheel diameter in meters
double diameter = 0.064;
//set global setpoints (later converted to ackerman-setpoints)
double linearSetpoint;    //m/s
double angularSetpoint;    //rad/s
int steeringPwm;   //PWM range (1000-2000µs)

//Define global setpoints for the motors and servo
double rSpeed;
double lSpeed;
int servoPWM;  // steering servo pwm (range 1000 - 2000 microseconds)

//Defing car geometry
const double wheelbase = 0.175;         //distance front axle - rear axle (meters)
const double trackWidth = 0.135;        //rear wheel track width (meters)

//Define Motor Struct
struct Motor{
  int in1, in2;
  int a, b;
  volatile long encoderCount;
  volatile long lastPosition;
  double setpoint, speedInput, pwmOutput;
  double motorSpeed;
  unsigned long lastTime;
  int pulsesPerRotation;
  PID pid;
  //speed sampling every 100ms - maybe change it later
  const unsigned long sampleTime = 100;

  Motor(int pin1, int pin2, int encA, int encB, double Kp, double Ki, double Kd, int pulsesPerRotation) 
    : in1(pin1), in2(pin2), a(encA), b(encB),
      encoderCount(0), lastPosition(0), 
      setpoint(0), speedInput(0), pwmOutput(0),
      motorSpeed(0), lastTime(0), pulsesPerRotation(pulsesPerRotation), pid(&speedInput, &pwmOutput, &setpoint, Kp, Ki, Kd, DIRECT) {}

  void attachInterrupts(void (*ISR)()) {
    pinMode(a, INPUT_PULLUP);
    pinMode(b, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(a), ISR, CHANGE);
  }

  void init() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255, 255); // PWM range - vielleicht nicht 100% so wie man's macht, aber ist verstandlich
  }

  //update speed (contains pid.compute() - needs to be called every loop iteration to do its job)
  void updateSpeed(double speedSetpoint){
    unsigned long currentTime = millis();
    setpoint = speedSetpoint;

    if (currentTime - lastTime >= sampleTime) {
      long currentPosition = encoderCount;
      speedInput = (((currentPosition - lastPosition) / (sampleTime / 1000.0)) / pulsesPerRotation) * (3.14*diameter); // Speed in meter/sec
      lastPosition = currentPosition;
      lastTime = currentTime;
    }

    pid.Compute();

    // Serial.print("Motor Speed Setpoint: "); Serial.print(setpoint);
    // Serial.print("Measured Speed: "); Serial.print(speedInput);
    // Serial.print("PWM Output: "); Serial.println(pwmOutput);

    //pwm output for speed control, both directions pwmOutput: [-255,255]
    if(pwmOutput > 0){
      analogWrite(in1, abs(pwmOutput));
      analogWrite(in2, 0);
    }
    else {
      analogWrite(in1, 0);
      analogWrite(in2, abs(pwmOutput));
    }
  }
};

//create Motor 1 and 2 !!!!CHECK CORRECT verschaltung!!!!! - zu viel Zeit fuer Debugging verschwendet :/
Motor rMotor(1, 2, 5, 6, 1000.0, 300.0, 0.0, 420);
Motor lMotor(3, 4, 15, 14, 1000.0, 300.0, 0.0, 420);
//define Servo
Servo steeringServo;
const int servoPin = 18;
const int servoCenterPWM = 1100; //servo center position pwm
const double maxSteeringAngle = 2* 0.349066;  // 2*20 deg in radians

void setup() {
  //initialize motors
  rMotor.init();
  rMotor.attachInterrupts(encoderISR1);
  lMotor.init();
  lMotor.attachInterrupts(encoderISR2);
  steeringServo.attach(servoPin);
  //serial port
  Serial.begin(115200);
  while (!Serial);  // Wait for serial connection
  Serial.println("Teensy Ready");
}

void loop() {
  // Read commands from ROS 2
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read line

    float linear = 0.0, angular = 0.0;

    // Parse received data (Format: "vX.X,oY.Y")
    int v_index = input.indexOf('v');
    int o_index = input.indexOf('o');

    if (v_index != -1 && o_index != -1) {
      linear = input.substring(v_index + 1, o_index - 1).toFloat();
      angular = input.substring(o_index + 1).toFloat();
    }
    linearSetpoint = linear;
    angularSetpoint = angular;
    ackerman(linearSetpoint, angularSetpoint);
    Serial.println(linear);
  }

  // Serial.println(steeringPwm);
  steeringServo.writeMicroseconds(steeringPwm);
  rMotor.updateSpeed(rSpeed);
  lMotor.updateSpeed(lSpeed);

}

//encoder interrup servic routine for encoder 1
void encoderISR1() {
  bool a = digitalRead(rMotor.a);
  bool b = digitalRead(rMotor.b);
  if (a == b) {
    rMotor.encoderCount++;
  } else{
    rMotor.encoderCount--;
  }
}

//encoder interrup servic routine for encoder 2
void encoderISR2() {
  bool a = digitalRead(lMotor.a);
  bool b = digitalRead(lMotor.b);
  if (a == b) {
    lMotor.encoderCount++;
  } else{
    lMotor.encoderCount--;
  }
}

void ackerman(double linear, double angular){
  rSpeed = linear + angular * (trackWidth/2);
  lSpeed = linear - angular * (trackWidth/2);
  double radians = atan((wheelbase * angular) / linear);
  radians = constrain(radians, -maxSteeringAngle, maxSteeringAngle);
  // Serial.println(radians);
  steeringPwm = servoCenterPWM + (int)(2*radians  * 500);
  Serial.println(steeringPwm);
}





