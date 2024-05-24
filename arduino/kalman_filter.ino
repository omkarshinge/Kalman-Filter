#include <Wire.h>
#define NUM_SAMPLES 2000

const int MPU6050_ADDR = 0x68;
const int PWR_MGMT_1 = 0x6B;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float accXMean, accYMean, accZMean;
float gyroRollVariance;
float gyroPitchVariance;
float gyroYawVariance;

// Arrays to store gyroscope data
float gyroRollSamples[NUM_SAMPLES];
float gyroPitchSamples[NUM_SAMPLES];
float gyroYawSamples[NUM_SAMPLES];

// Arrays to store accelerometer data
float accXSamples[NUM_SAMPLES];
float accYSamples[NUM_SAMPLES];
float accZSamples[NUM_SAMPLES];

int16_t AccXLSB;
int16_t AccYLSB;
int16_t AccZLSB;

int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

float accXCalibration = 0.0;
float accYCalibration = 0.0;
float accZCalibration = 0.0;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float QRoll, QPitch;

uint32_t LoopTimer;

float KalmanAngleRoll=0, kErrorAngleRoll= 0;
float KalmanAnglePitch=0, kErrorAnglePitch= 0;
float KalmanAngleYaw=0, KalmanUncertaintyAngleYaw= 0;
float KalmanRollOutput[]={0,0};
float KalmanPitchOutput[]={0,0};

float gyroVariance = 1;
float accelerometerVariance = 0.000001;

float delT = 0.004;
// float Q = delT * delT * gyroVariance;
float mError = accelerometerVariance;

float KG = 0;

void kalmanFilter(float* KalmanOutput, float X, float eError, float KalmanInput, float M, float Q) {
  // Predict
  X = X + delT * KalmanInput;
  eError = eError + Q;

  // Update
  KG = eError/(eError + mError);
  X = X + KG * (M - X);
  eError = (1 - KG) * eError;

  KalmanOutput[0]=X; 
  KalmanOutput[1]=eError;
}

float calculateMean(float data[], int length) {
  float sum = 0.0;
  for (int i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum / length;
}

float calculateVariance(float data[], int length, float mean) {
  float sum = 0.0;
  for (int i = 0; i < length; i++) {
    sum += (data[i] - mean) * (data[i] - mean);
  }
  return sum / length;
}

void gyro_signals(void) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(); 

  Wire.requestFrom(MPU6050_ADDR,6);
  AccXLSB = Wire.read() << 8 | Wire.read();
  AccYLSB = Wire.read() << 8 | Wire.read();
  AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(MPU6050_ADDR,6);
  GyroX=Wire.read()<<8 | Wire.read();
  GyroY=Wire.read()<<8 | Wire.read();
  GyroZ=Wire.read()<<8 | Wire.read();

  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;

  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);

  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPU6050_ADDR); 
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(250);
  Serial.println("\nCalibrating Gyroscope & Accelerometer");

  for (RateCalibrationNumber=0; RateCalibrationNumber<NUM_SAMPLES; RateCalibrationNumber ++) {
    gyro_signals();
    gyroRollSamples[RateCalibrationNumber] = RateRoll;
    gyroPitchSamples[RateCalibrationNumber] = RatePitch;
    gyroYawSamples[RateCalibrationNumber] = RateYaw;
    accXSamples[RateCalibrationNumber] = AccX;
    accYSamples[RateCalibrationNumber] = AccY;
    accZSamples[RateCalibrationNumber] = AccZ;
    delay(10);
  }

  RateCalibrationRoll = calculateMean(gyroRollSamples, NUM_SAMPLES);
  RateCalibrationPitch = calculateMean(gyroPitchSamples, NUM_SAMPLES);
  RateCalibrationYaw = calculateMean(gyroYawSamples, NUM_SAMPLES);

  gyroRollVariance = calculateVariance(gyroRollSamples, NUM_SAMPLES, RateCalibrationRoll);
  gyroPitchVariance = calculateVariance(gyroPitchSamples, NUM_SAMPLES, RateCalibrationPitch);
  gyroYawVariance = calculateVariance(gyroYawSamples, NUM_SAMPLES, RateCalibrationYaw);

  accXMean = calculateMean(accXSamples, NUM_SAMPLES);
  accYMean = calculateMean(accYSamples, NUM_SAMPLES);
  accZMean = calculateMean(accZSamples, NUM_SAMPLES);

  accXCalibration = accXMean;
  accYCalibration = accYMean;
  accZCalibration = 1 - accZMean;

  float accXVariance = calculateVariance(accXSamples, NUM_SAMPLES, accXMean);
  float accYVariance = calculateVariance(accYSamples, NUM_SAMPLES, accYMean);
  float accZVariance = calculateVariance(accZSamples, NUM_SAMPLES, accZMean);

  Serial.println("Gyroscope Mean & Variance:");
  Serial.print("Mean Roll: "); Serial.print(RateCalibrationRoll);
  Serial.print("\t Variance Roll: "); Serial.println(gyroRollVariance, 6);
  Serial.print("Mean Pitch: "); Serial.print(RateCalibrationPitch); 
  Serial.print("\t Variance Pitch: "); Serial.println(gyroPitchVariance, 6);
  Serial.print("Mean Yaw: "); Serial.print(RateCalibrationYaw); 
  Serial.print("\t Variance Yaw: "); Serial.println(gyroYawVariance, 6);

  Serial.println("\nAccelerometer Mean & Variance:");
  Serial.print("Mean X: "); Serial.print(accXMean); 
  Serial.print("\t Variance X: "); Serial.println(accXVariance, 6);
  Serial.print("Mean Y: "); Serial.print(accYMean); 
  Serial.print("\t Variance Y: "); Serial.println(accYVariance, 6);
  Serial.print("Mean Z: "); Serial.print(accZMean); 
  Serial.print("\t Variance Z: "); Serial.println(accZVariance, 6);

  QRoll = delT*delT*gyroRollVariance;
  QPitch = delT*delT*gyroPitchVariance;

  LoopTimer=micros();
}

void loop() {

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any leading or trailing whitespace
    
    if (command.equalsIgnoreCase("RESET")) {
      Serial.println("Resetting ...");
      delay(100); // Small delay to ensure the message is sent
      ESP.restart();
    } else {
      Serial.println("Unknown command. Send 'RESET' to restart the NodeMCU.");
    }
  }

  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  kalmanFilter(KalmanRollOutput, KalmanAngleRoll, kErrorAngleRoll, RateRoll, AngleRoll, QRoll);
  KalmanAngleRoll=KalmanRollOutput[0];
  kErrorAngleRoll=KalmanRollOutput[1];

  kalmanFilter(KalmanPitchOutput, KalmanAnglePitch, kErrorAnglePitch, RatePitch, AnglePitch, QPitch);
  KalmanAnglePitch=KalmanPitchOutput[0]; 
  kErrorAnglePitch=KalmanPitchOutput[1];

  Serial.print(AngleRoll);
  Serial.print("\t");
  Serial.print(KalmanAngleRoll);
  Serial.print("\t");

  Serial.print(AnglePitch);
  Serial.print("\t");
  Serial.println(KalmanAnglePitch);

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}