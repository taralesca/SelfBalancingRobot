#include "I2Cdev.h"
#include "PID_v1.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define INT_FIFO_OVERFLOW 0x10
#define INT_DATA_READY    0x02
/*
 * Pins connected to L298N must have PWM.
 */
const int L298N_IN1 = 6;  // +
const int L298N_IN2 = 9;  // -
const int L298N_IN3 = 10; // +
const int L298N_IN4 = 11; // -

/* 
 * The following OFFSETs vary between each MPU6050 unit.
 * Check documentation for instructions on how to set these.
 */ 
const int X_GYRO_OFFSET = 100;
const int Y_GYRO_OFFSET = -20;
const int Z_GYRO_OFFSET = 45;
const int Z_ACCE_OFFSET = 1288;

/*
 * To correctly set the balance point, hold the robot in the desired position.
 * Disconnect the motors and check the Serial Monitor for the right value.
 */
double balancePoint = 177;

/*
 * PID allows the motors to respond with the right amount of torque depending on
 * MPU6050 output.
 * To calibrate PID, start with all three values at 0.
 */ 
const double P = 24.5; // Increase this FIRST until the robot oscillates constantly.
const double D = 1.4;  // Increase this SECOND until the oscillations stop.
const double I = 220;  // Increase this THIRD until the robot responds well to being pushed.


/*
 * The algorithm works like this:
 * MPU => inputVal => [PID Calculation] => responseVal
 */
double inputVal, responseVal;

PID pidControl(&inputVal, &responseVal, &balancePoint, P, I, D, DIRECT);


Quaternion quat;     // [w, x, y, z] - MPU6050 outputs Quaternions
VectorFloat gravity; //    [x, y, z] - Quaternions will be converted to a gravity vector

 
// MPU6050 communicates with the board by interrupts on digital pin 2.
bool mpuInterrupt;
void dmpDataReady() {
    mpuInterrupt = true;
}

class Input {
  public:
    MPU6050 mpu;

    float ypr[3];            // Yaw, Pitch, Roll [tilt control]
    int packetSize;
    bool dmpReady;           // Digital Motion Processor has 2 states: ready/not ready

    uint8_t mpuIntStatus;    // 0 = no interrupt

    uint16_t fifoCount;      // Number of bytes in buffer.
    uint8_t fifoBuffer[64];  

    void init() {
        mpu.initialize();

        mpu.setXGyroOffset(X_GYRO_OFFSET);
        mpu.setYGyroOffset(Y_GYRO_OFFSET);
        mpu.setZGyroOffset(Z_GYRO_OFFSET);
        mpu.setZAccelOffset(Z_ACCE_OFFSET);

        if(mpu.dmpInitialize() == 0) {
            mpu.setDMPEnabled(true);
            
            attachInterrupt(0, dmpDataReady, RISING); // Interrupt pin 0 = Arduino digital pin 2

            mpuIntStatus = mpu.getIntStatus();
            dmpReady = true; // DMP becomes ready if successfully initialized
            packetSize = mpu.dmpGetFIFOPacketSize();
            
            pidControl.SetMode(AUTOMATIC);
            pidControl.SetSampleTime(10);
            pidControl.SetOutputLimits(-255, 255);
        }
    }
    
} input;

class MotorControl {
    // Adjust the following pins according to your L298N configuration:
    static const int MOT1_IN1 = L298N_IN1;
    static const int MOT1_IN2 = L298N_IN2;

    static const int MOT2_IN1 = L298N_IN3;
    static const int MOT2_IN2 = L298N_IN3;

  public:

    void init() {
        // Stop both motors initially.
        analogWrite(MOT1_IN1, LOW);
        analogWrite(MOT1_IN2, LOW);

        analogWrite(MOT2_IN1, LOW);
        analogWrite(MOT2_IN2, LOW);
    }

    void moveForward() {
        analogWrite(MOT1_IN1, responseVal);
        analogWrite(MOT1_IN2, LOW);

        analogWrite(MOT2_IN1, responseVal);
        analogWrite(MOT2_IN2, LOW);
    }

    void moveBackwards() {
        analogWrite(MOT1_IN1, LOW);
        analogWrite(MOT1_IN2, -responseVal);

        analogWrite(MOT2_IN1, LOW);
        analogWrite(MOT2_IN2, -responseVal);
    }

    void stopBoth() {
        analogWrite(MOT1_IN1, LOW);
        analogWrite(MOT1_IN2, LOW);
        analogWrite(MOT2_IN1, LOW);
        analogWrite(MOT2_IN2, LOW);
    }

} motors;


void setup() {

        pinMode (L298N_IN1, OUTPUT);
        pinMode (L298N_IN2, OUTPUT);
        pinMode (L298N_IN3, OUTPUT);
        pinMode (L298N_IN4, OUTPUT);

        // Uncomment the following line for debugging purposes.
        //~ Serial.begin(115200);

        input.init();
        motors.init();
}

void loop () {
    // Only run code if the DMP is in the ready state
    if(!input.dmpReady) return;

    while(!mpuInterrupt && input.fifoCount < input.packetSize) {
        pidControl.Compute();

        // Uncomment the following line for debugging purposes.
        //~ Serial.print(inputVal); Serial.print(" =>"); Serial.println(responseVal);

        if(inputVal > 150 && inputVal < 200) { // Is the robot leaning towards
            if(responseVal > 0) { // front?
                motors.moveForward();
            } else if (responseVal < 0) { // or back?
                motors.moveBackwards();
            }
        } else { // Is the robot perfectly balanced?
            motors.stopBoth();
        }
    }

    mpuInterrupt = false;
    input.mpuIntStatus = input.mpu.getIntStatus();
    input.fifoCount = input.mpu.getFIFOCount();

    if((input.mpuIntStatus & INT_FIFO_OVERFLOW) || input.fifoCount == 1024) {
        // Interrupt occurs if FIFO is full
        input.mpu.resetFIFO();
    } else if (input.mpuIntStatus & INT_DATA_READY) {
            // Interrupt occurs for recieving fresh data from MPU6050
            while (input.fifoCount < input.packetSize) {
                input.fifoCount = input.mpu.getFIFOCount(); 
            }

            input.mpu.getFIFOBytes(input.fifoBuffer, input.packetSize);
            input.fifoCount -= input.packetSize;

            // Get Quaternion from MPU6050
            input.mpu.dmpGetQuaternion(&quat, input.fifoBuffer); 
            // Turn it into gravity vector
            input.mpu.dmpGetGravity(&gravity, &quat); 
            // Turn gravity into Yaw, Pitch, Roll
            input.mpu.dmpGetYawPitchRoll(input.ypr, &quat, &gravity); 

            // We use the Pitch to check for forward/backward movement
            inputVal = input.ypr[1] * 180/M_PI + 180;
    }
}