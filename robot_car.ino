#include <QuadratureEncoder.h>

// PIN CONSTS
Encoders rightEncoder(8,9); // right encoder pins
Encoders leftEncoder(3,4);  // left encoder pins
const int RF = 11;          // right forward pin
const int RB = 10;          // right backward pin

const int LB = 5;           // left backward pin
const int LF = 6;           // left forward pin


// KEY: [slow value]S; [fast value]F

// DELAY CONSTS

// [initial value] + [total time needed to delay] / [# of movements - 1]

const int AFTER_DELAY = 100; // 200 S; 100 F: delay in ms after each movement

// SPEED CONSTS - STRAIGHT
const int MIN_SPEED_STR = 160;      // 100 S; 160 F: starting motor speed
const int MAX_SPEED_STR = 255;      // 200 S; 255 F: max motor speed
const int END_SPEED_STR = 100;      // 100: ending motor speed
const int SLOWDOWN_SPEED_STR = 40;  // 40: speed to slow down ahead motor by
const int ACCEL_STEP_STR = 10;       // 5 S; 10 F: increment of speed value every 10 ms in initial acceleration

// SPEED CONSTS - TURN
const int MIN_SPEED_TURN = 70;     // 70  S; 100 F: starting motor speed
const int MAX_SPEED_TURN = 120;     // 80  S; 150 F: max motor speed
const int END_SPEED_TURN = 65;     // 65  S; 100 F: ending motor speed
const int SLOWDOWN_SPEED_TURN = 40; // 40: speed to slow down ahead motor by
const int ACCEL_STEP_TURN = 1;      // 1: increment of speed value every 10 ms in initial acceleration

// MOTOR SPECIFIC CONSTS; DO NOT TOUCH UNLESS ONE MOTOR IS ACTING WEIRD
const int LEFT_OFFSET = 0;     // 0: left motor offset value
const int RIGHT_OFFSET = 0;    // 0: right motor offset value

bool delayLeft = false;        // false: delay left motor until right turns; use if left starts way faster
bool delayRight = false;       // false: delay right motor until left turns; use if right starts way faster
const int DELAY_THRESHOLD = 5; // 5: threshold for activation of delayed motors

// STEP CONSTS
const int SLOW_DIST = 400;     // 600 S; 400 F: distance at which deceleration begins
// const int TURN_90_STEPS_L = 551; // 551 551 551 551 551: steps for a 90 deg turn: testing still needed
// const int TURN_90_STEPS_R = 551; // 551 551 551 551 551: steps for a 90 deg turn: testing still needed

const int TURN_90_STEPS_L = 563;
const int TURN_90_STEPS_R = 556;

// STATE VARIABLES
bool check = true;   // should check for button down press?

bool moving = false; // has moving begun?
long targetL = 0;    // target left encoder pos
long targetR = 0;    // target right encoder pos


void setup() {
  // set up pin outputs, turn LED on to signal ready
  pinMode(A0, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(LF, OUTPUT);
  Serial.begin(9600);
  stop();
}

/* USABLE METHODS: fwDist(cm); bwDist(cm); tr90(); tl90(); trDeg(deg); tlDeg(deg) */

void buttonSetup() {
  // put run code here
}

// HIGH LEVEL MOVEMENT - move cm/degrees
void tr90() {
  tr(TURN_90_STEPS_R);
}

void tl90() {
  tl(TURN_90_STEPS_L);
}

void fwDiag(int cm) {
  fwDist((int)(1.414 * cm));
}

void bwDiag(int cm) {
  bwDist((int)(1.414 * cm));
}

void fwDist(int cm) {
  fw((int)(42.3860315902246 * (long double)cm + 1.46322153487199));
}

void bwDist(int cm) {
  bw((int)(42.3860315902246 * (long double)cm + 1.46322153487199));
}

void trDeg(int deg) {
  tr((int)((double)TURN_90_STEPS_R * deg / 90));
}

void tlDeg(int deg) {
  tl((int)((double)TURN_90_STEPS_L * deg / 90));
}

// LOW LEVEL MOVEMENT - move # of steps
void tr(int steps) {
  mv(false, true, steps, MIN_SPEED_TURN, MAX_SPEED_TURN, SLOWDOWN_SPEED_TURN, END_SPEED_TURN, ACCEL_STEP_TURN);
}

void tl(int steps) {
  mv(true, false, steps, MIN_SPEED_TURN, MAX_SPEED_TURN, SLOWDOWN_SPEED_TURN, END_SPEED_TURN, ACCEL_STEP_TURN);
}

void fw(int steps) {
  mv(true, true, steps, MIN_SPEED_STR, MAX_SPEED_STR, SLOWDOWN_SPEED_STR, END_SPEED_STR, ACCEL_STEP_STR);
}

void bw(int steps) {
  mv(false, false, steps, MIN_SPEED_STR, MAX_SPEED_STR, SLOWDOWN_SPEED_STR, END_SPEED_STR, ACCEL_STEP_STR);
}

// MAIN MOVEMENT CODE
void mv(bool RFW, bool LFW, int steps, int MIN_SPEED, int MAX_SPEED, int SLOWDOWN_SPEED, int END_SPEED, int ACCEL_STEP) {
  long startL = targetL;
  long startR = targetR;

  long trueStartL = leftEncoder.getEncoderCount();
  long trueStartR = rightEncoder.getEncoderCount();

  long distL = abs(leftEncoder.getEncoderCount() - startL);
  long distR = abs(rightEncoder.getEncoderCount() - startR);
  long trueDistL = abs(leftEncoder.getEncoderCount() - trueStartL);
  long trueDistR = abs(rightEncoder.getEncoderCount() - trueStartR);

  long speedL = MIN_SPEED + LEFT_OFFSET;
  long speedR = MIN_SPEED + RIGHT_OFFSET;
  long lastTime = millis();

  targetL += LFW ? steps : -steps;
  targetR += RFW ? steps : -steps;

  bool RFE = RFW;
  bool RBE = !RFW;
  bool LFE = LFW;
  bool LBE = !LFW;

  while (distL < steps && distR < steps) {
// Get distance values
    distL = abs(leftEncoder.getEncoderCount() - startL);
    distR = abs(rightEncoder.getEncoderCount() - startR);

    trueDistL = abs(leftEncoder.getEncoderCount() - trueStartL);
    trueDistR = abs(rightEncoder.getEncoderCount() - trueStartR);

// Write motor values
    if (distL > distR) {
if (trueDistL > DELAY_THRESHOLD || !delayRight) {
      analogWrite(RF, RFE ? speedR : 0);
      analogWrite(RB, RBE ? speedR : 0);
}

if (trueDistR > DELAY_THRESHOLD || !delayLeft) {
      analogWrite(LB, LBE ? speedL - SLOWDOWN_SPEED : 0);
      analogWrite(LF, LFE ? speedL - SLOWDOWN_SPEED : 0);
}
    }
    else if (distL < distR) {
if (trueDistL > DELAY_THRESHOLD || !delayRight) {
      analogWrite(RF, RFE ? speedR - SLOWDOWN_SPEED : 0);
      analogWrite(RB, RBE ? speedR - SLOWDOWN_SPEED : 0);
}

if (trueDistR > DELAY_THRESHOLD || !delayLeft) {
      analogWrite(LB, LBE ? speedL : 0);
      analogWrite(LF, LFE ? speedL : 0);
}
    }
    else {
if (trueDistL > DELAY_THRESHOLD || !delayRight) {
      analogWrite(RF, RFE ? speedR : 0);
      analogWrite(RB, RBE ? speedR : 0);
}

if (trueDistR > DELAY_THRESHOLD || !delayLeft) {
      analogWrite(LB, LBE ? speedL : 0);
      analogWrite(LF, LFE ? speedL : 0);
}
    }

// Update speed
    if (abs(distL - steps) < SLOW_DIST || abs(distR - steps) < SLOW_DIST) {
      int baseSpeed = (MAX_SPEED - END_SPEED) * (abs(distR - steps) + abs(distR - steps))/(2 * SLOW_DIST) + END_SPEED;
      speedL = baseSpeed + LEFT_OFFSET;
      speedR = baseSpeed + RIGHT_OFFSET;
    }
    else if (millis() - lastTime > 10) {
      if (speedL < MAX_SPEED) {
        speedL += ACCEL_STEP;
        if (speedL > MAX_SPEED) speedL = MAX_SPEED;
      }
      if (speedR < MAX_SPEED) {
        speedR += ACCEL_STEP;
        if (speedR > MAX_SPEED) speedR = MAX_SPEED;
      }
      lastTime = millis();
    }
  }

  // TEST: if target ~ encoder, works well <- !!!!
  // ELSE: tune, might not be able to recorrect previous movement
  Serial.print(targetL);
  Serial.print("L; ");
  Serial.println(leftEncoder.getEncoderCount());
  Serial.print(targetR);
  Serial.print("R; ");
  Serial.println(rightEncoder.getEncoderCount());

  stop();
  delay(AFTER_DELAY);
}

void stop() {
  // stop all motors
  analogWrite(RF, 0);
  analogWrite(RB, 0);

  analogWrite(LB, 0);
  analogWrite(LF, 0);
}

void loop() {
  // look for button pressed down
  if (check) {
    if (analogRead(A0) > 512) { digitalWrite(LED_BUILTIN, LOW); delay(1000); check = false; buttonSetup(); }
    return;
  }
}

