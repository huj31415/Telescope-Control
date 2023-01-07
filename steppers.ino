#include <AccelStepper.h>

#define PitchDir 8
#define PitchStep 9
#define YawDir 6
#define YawStep 7
#define StepsPerRevolution 200

// Resolution pins (common)
#define M0 10
#define M1 16
#define M2 14

#define GearRatio 10  // number of motor revolutions per telescope revolution

#define SiderealDay 86164.0905  //seconds

#define StepRes 32

#define threshold 4 // joystick detection threshold
#define Accel 100*StepRes   // joystick response acceleration

// joystick values
int xVal;
int yVal;
bool joystick = false;


unsigned long currentTime; //sec

// const double pitchSpeed = (StepsPerRevolution / 2 * 32) / (SiderealDay / 2) * GearRatio;  // 0.742768822 microsteps/sec

// // cumulative angles
// double pitchCumAngle;
// double yawCumAngle;

AccelStepper pitch(AccelStepper::DRIVER, PitchStep, PitchDir);
AccelStepper yaw(AccelStepper::DRIVER, YawStep, YawDir);

struct Coord {
  double alt;
  double az;
};

Coord currentPos;
Coord polaris;

void setResolution(int res = 32) {
  int x = log(res) / log(2);
  digitalWrite(M0, (x & 1) == 0 ? LOW : HIGH);
  digitalWrite(M1, (x & 2) == 0 ? LOW : HIGH);
  digitalWrite(M2, (x & 4) == 0 ? LOW : HIGH);
}

void getPole() {
  // aim at Polaris and store angles
  polaris.alt = 0;
  polaris.az = 0;
}

void listenToJoystick(AccelStepper *stepper, int delta) {
  delta = map(delta, 0, 1023, -100*StepRes, 100*StepRes);
  if (abs(delta) >= threshold*StepRes) {
    stepper->move(delta);
    Serial.println("moving by " + String(delta));
  } else if (stepper->isRunning()) {
    Serial.println("preparing to stop");
    stepper->stop();
    stepper->setAcceleration(10000);
    stepper->runToPosition();
    stepper->setAcceleration(Accel);
    Serial.println("Stopped");
  }
}

void joystickControl(int dx, int dy) {
  listenToJoystick(&pitch, dy);
  listenToJoystick(&yaw, dx);
}


void track(long seconds) {
  unsigned long begin = currentTime;
  while (currentTime - begin <= seconds) {
    currentTime = millis() / 1000;
  }
}

void setup() {
  Serial.begin(9600);
  setResolution(StepRes);
  pitch.setMaxSpeed(1000*StepRes);
  pitch.setAcceleration(Accel);
  yaw.setMaxSpeed(1000*StepRes);
  yaw.setAcceleration(Accel);
  // pitch.setSpeed(pitchSpeed);  // steps/sec

}

void loop() {
  xVal = analogRead(A0);
  yVal = analogRead(A1);
  joystickControl(xVal, yVal);
  pitch.run();
  yaw.run();
  //pitch.runSpeed();
  //yaw.runSpeed();
}
