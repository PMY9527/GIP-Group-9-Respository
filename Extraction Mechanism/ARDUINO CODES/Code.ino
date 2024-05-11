#include <Wire.h>
#include <AS5600.h>
#include <AccelStepper.h>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include <Stepper.h>
using namespace BLA;
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif


AMS_5600 ams5600;

float lastAngle = 0.0;
int totalRevolutions = 0;
float totalAngle = 0.0;
float startAngle = 0.0;
float totalStep = 0;
float e = 0;
float u = 0;
unsigned long previousMillis = 0;
float VerticalTarget = 0;
int VerticalIndex = 0;
const long interval = 50;
int stepsPerRevolution = 200;
int sole = 12; 

// ********** THE ANGLE ARRAY ********** //
float VerticalAngle[] = {200, 50, 0}; // READINGS FROM OCR, ENDS WITH 0 FOR HOMING.
float HorizontalAngle[] = {25, 50, 0}; 
// ********** THE STATE-SPACE MATICES ********** //
BLA::Matrix<2,2> A = {1,0.0035,0.00000042,0.77}; // tstep 0.003s
BLA::Matrix<2,1> B = {0.000324,0.155};
BLA::Matrix<1,2> H = {1,0};
// ********** THE STATE-SPACE MATICES ********** //

// ********** KALMAN FILTER VARIABLES ********** //
float R = 0.5; // MEASUREMENT NOISE 
BLA::Matrix<2,2> Q = {0.0001,0,0,0.0001}; //PROCESS NOISE
BLA::Matrix<2,2> I = {1,0,0,1};
float filtered_step = 0;
BLA::Matrix<2,1> x_hat_k_minus_1 = {0,0}; // INITIAL ESTIMATES FOR THE STATE <x1_hat_k-1>
BLA::Matrix<2,2> P_k_minus_1 = {0,0,0,0};  // INITIAL ERROR COVARIANCE <P_k-1>
BLA::Matrix<2, 1> x_hat_k;
Matrix<1,1,float> u_mat;
// ********** KALMAN FILTER VARIABLES ********** //

float numAngle = sizeof(VerticalAngle) / sizeof(VerticalAngle[0]);
int max = 300; // MAXIMUM SPEED NOT STALLING 
float kp = 20; // PROPORTIONAL GAIN

Stepper myStepperH(stepsPerRevolution, A0, A1, A2, A3); // HORIZONTAL 
AccelStepper myStepperL(AccelStepper::FULL4WIRE, 4, 5, 6, 7); // VERTICAL
AccelStepper myStepperR(AccelStepper::FULL4WIRE, 8, 9, 10, 11);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  startAngle = ams5600.getRawAngle();
  myStepperL.setMaxSpeed(max);
  myStepperR.setMaxSpeed(max);
  myStepperH.setSpeed(200);
  pinMode(sole, OUTPUT); // SOLENOID
}

float convertRawAngleToDegrees(word newAngle) {
  float retVal = newAngle * 0.087890625;
  return retVal;
}

void loop() {

  // ********** ANGLE MEASUREMENTS CONVERSION ********** //
  float rawAngle = ams5600.getRawAngle();
  float currentAngleDegrees = convertRawAngleToDegrees(rawAngle);
  float startAngleDegrees = convertRawAngleToDegrees(startAngle);
  float correctedAngle = currentAngleDegrees - startAngleDegrees;
  unsigned long currentMillis = millis();

  if (correctedAngle < 0) {
    correctedAngle += 360;
  }

  if (lastAngle > 300.0 && correctedAngle < 60.0) {  // CLOCKWISE PAST ZERO
    totalRevolutions++;
  } else if (lastAngle < 60.0 && correctedAngle > 300.0) {  // COUNTERCLOCK WISE PAST ZERO
    totalRevolutions--;
  }

  totalAngle = totalRevolutions * 360.0 + correctedAngle;
  totalStep = totalAngle / 1.8;  // ANGLE ---> STEPS
  Matrix<1,1,float> totalStep_Mat = {totalStep};
  // ********** ANGLE MEASUREMENTS CONVERSION ********** //

  if (millis() >= 1000) { 
    VerticalTarget = VerticalAngle[VerticalIndex];

    // e = VerticalTarget - filtered_step;
    // u = kp * e;
    // u = constrain(u, -max, max); 
    float Kr = 23.04;
    Matrix<1, 2> K = {23.03, 0.352};
    Matrix<1,1,float> ref_matrix = {VerticalTarget * Kr};
    u_mat = {ref_matrix - K * x_hat_k};
    u = u_mat(0,0);
    u = constrain(u, -max, max); 
  }

  // ********** KALMAN FILTER ********** //

  // *** 1.PREDICT STATE AND ERROR COVARIANCE *** //
  Matrix<2,1> x_hat_k_predict = A * x_hat_k_minus_1 + B * u;
  Matrix<2,2> P_k_predict = A * P_k_minus_1 * (~A) + Q;

  // *** 2. COMPUTE KALMAN GAIN *** //
  Matrix<2,1> K_k = P_k_predict * (~H) * Inverse(H * P_k_predict *(~H) + R);

  // *** 3. COMPUTE THE ESTIMATE *** //
  x_hat_k = x_hat_k_predict + K_k * (totalStep_Mat - H * x_hat_k_predict);
  filtered_step = x_hat_k(0,0);
  //float filtered_velocity = x_hat_k(1,0);
  // *** 4. COMPUTE THE ERROR COVARIANCE *** //
  Matrix<2,2> P_k = (I - K_k * H)*P_k_predict;
  
  // *** 5. UPDATE FOR NEXT TIME STEP *** //
  x_hat_k_minus_1 = x_hat_k;
  P_k_minus_1 = P_k;
  // ********** KALMAN FILTER ********** //


  // ********** PROPORTIONAL CONTROL ********** //
  myStepperL.setSpeed(u);
  myStepperR.setSpeed(u);
  myStepperR.runSpeed();
  myStepperL.runSpeed();
  // ********** PROPORTIONAL CONTROL ********** //


  // ********** PRINT RESULTS ********** //
  Serial.print(millis());
  Serial.print(", ");
  Serial.print(u);
  Serial.print(", ");
  Serial.print(VerticalTarget);
  Serial.print(", ");
  Serial.print(totalStep);
  Serial.print(", ");
  Serial.println(filtered_step);
  // Serial.println(filtered_velocity);
  // ********** PRINT RESULTS ********** //


  // ********** HORIZONTAL & PUNCHING ********** // 
      // if (currentMillis - previousMillis >= interval) {
      //   previousMillis = currentMillis; 
      //   VerticalIndex++;
      // }
  if (VerticalIndex <= numAngle - 1 && abs(e) < 0.5 ) {
      myStepperH.step(HorizontalAngle[VerticalIndex]);
      digitalWrite(sole,HIGH);
      delay(20);
      digitalWrite(sole,LOW);
      delay(1000);
      digitalWrite(sole,HIGH);
      delay(20);
      digitalWrite(sole,LOW);
      delay(1000);

      VerticalIndex++; 
      previousMillis = currentMillis;
    }
   else if (VerticalIndex > numAngle - 1) { }
    // FINISH EXTRACTION, STOP EVERYTHING.
  lastAngle = correctedAngle; // FOR ANGLE MEASUREMENT
  // ********** HORIZONTAL & PUNCHING ********** // 
}
