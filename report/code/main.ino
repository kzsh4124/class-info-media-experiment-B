#define ML1 2
#define ML2 3
#define MR1 1
#define MR2 4
#define SENSOR_PIN 0
#define ENCODER_L 5
#define ENCODER_R 10
#define LED 13

#define RIGHT 0
#define LEFT 1

#define SENSOR_DISABLED false

float gearRatio;
float wheelDiameter;
float distancePerMotorRotate;

int ENCODER_PIN[2] = {ENCODER_R, ENCODER_L};

int isActive = true;

float currentSpeed[2] = {0,0};

//original map function which can map with float values
float mapFloat(float InputValue, float InputLower, float InputUpper, float OutputLower, float OutputUpper){
  float value = (OutputUpper-OutputLower)*(InputValue - InputLower) / (InputUpper - InputLower) + OutputLower;
  return max(min(value, OutputUpper), OutputLower);
}

//motor power control function
void controlMotor(float Rspeed, float Lspeed){ //argument range -1 to 1
  if(Rspeed>=0){
    analogWrite(MR1, int(255*Rspeed));
    analogWrite(MR2, 0);
  }else{
    analogWrite(MR2, int(255*(-Rspeed)));
    analogWrite(MR1, 0);
  }

  if(Lspeed>=0){
    analogWrite(ML1, int(255*Lspeed));
    analogWrite(ML2, 0);
  }else{
    analogWrite(ML2, int(255*(-Lspeed)));
    analogWrite(ML1, 0);
  }
}

//chattering eliminator function for the binary hall sensors 
int prevEncoderState[2] = {0,0};
int _smoothEncoderSignal(int encoderNum){
  while(true){
    int encoderSignals[3] = {0,0,0};
    for(int i = 0; i < 3; i++){
      encoderSignals[i] = digitalRead(ENCODER_PIN[encoderNum]);
    }
    if(encoderSignals[0]==encoderSignals[1] && encoderSignals[1]==encoderSignals[2]){
      return encoderSignals[0];  
    }
  }
}

//signal state detection function
bool _isChanged(int encoderNum){ //argument range 0 or 1
  int encoderState = _smoothEncoderSignal(encoderNum);
  bool changed = false;
  if(prevEncoderState[encoderNum]==0 && encoderState==1){
    changed = true;
  }
  prevEncoderState[encoderNum] = encoderState;
  return changed;
}

//measure speed with parallel processing emulation (amazingly faster than before)
int deadLineTime = 50000;
int pulsePerRotate = 11;
int prevSignalTime[2] = {0,0};
float _getSpeedWithTime(int motorNum){
  int nowTime = micros();
  int deltaTime = nowTime - prevSignalTime[motorNum];
  if(_isChanged(motorNum)){
    float MotorFreq = 1000000/(deltaTime*pulsePerRotate);
    float speed = MotorFreq*distancePerMotorRotate;//cm/s
    prevSignalTime[motorNum] = nowTime;
    return speed;
  }else if(deltaTime > deadLineTime){
    return 0;
  }else{
    return currentSpeed[motorNum];
  }
}

//speed update function
void getSpeedWithTime(float *spd){
  spd[RIGHT] = _getSpeedWithTime(RIGHT);
  spd[LEFT] = _getSpeedWithTime(LEFT);
}

//smooth start speed function
void delayAccelerationSpeed(float targetSpeed, float *newTargetSpeed, float maxError = 10){
  float speedDeltas[] = {targetSpeed-currentSpeed[RIGHT], targetSpeed-currentSpeed[LEFT]};
  if(speedDeltas[0] >= maxError && speedDeltas[1] >= maxError){
    newTargetSpeed[RIGHT] = min(currentSpeed[RIGHT] + maxError, targetSpeed);
    newTargetSpeed[LEFT] = min(currentSpeed[LEFT] + maxError, targetSpeed);
  }else{
    newTargetSpeed[RIGHT] = targetSpeed;
    newTargetSpeed[LEFT] = targetSpeed;
  }
  return;
}

float getLinePos(){
  float linePos = mapFloat(float(analogRead(SENSOR_PIN)),7.0f,673.0f,-10.0f,10.0f);
  return linePos;
}

float sensorErrorPrev;
float sensorErrorSum;
float getPIDsensorValue(int linePos){
  float skP = 10.0;
  float skI = 0.0002;
  float skD = 12.5;

  //on start
  if(currentSpeed[RIGHT]<=30 && currentSpeed[LEFT]<=30){
    skP = 4.5;
    skI = 0.00015;
    skD = 8.0;

  }
  float error = linePos;
  sensorErrorSum += error;
  float PIDvalue = error*skP + sensorErrorSum*skI + (error - sensorErrorPrev)*skD;
  sensorErrorPrev = error;
  return PIDvalue;
}


float spdPrev[2]; 
float spdErrorPrev[2];
float spdErrorSum[2];
float power[2];
void controlEachMotorWithPID(float rightTargetSpeed, float leftTargetSpeed){
  float mkP = 0.04;
  float mkI = 0.000015;
  float mkD = 0.05;
  
  float spdError[2];
  
  spdError[RIGHT] = rightTargetSpeed - currentSpeed[RIGHT];
  spdError[LEFT] = leftTargetSpeed - currentSpeed[LEFT];
  spdErrorSum[RIGHT] += spdError[RIGHT];
  spdErrorSum[LEFT] += spdError[LEFT];
  float pwrR = spdError[RIGHT]*mkP + (spdError[RIGHT]-spdErrorPrev[RIGHT])*mkD + spdErrorSum[RIGHT]*mkI;
  float pwrL = spdError[LEFT]*mkP + (spdError[LEFT]-spdErrorPrev[LEFT])*mkD + spdErrorSum[LEFT]*mkI;
  spdErrorPrev[RIGHT] = spdError[RIGHT];
  spdErrorPrev[LEFT] = spdError[LEFT];
  controlMotor(pwrR, pwrL);
}

float limitSpeedOnCorner(float targetSpeed, float deacceleratedSpeed, int linePositon){
  float spdDelta = (targetSpeed - deacceleratedSpeed)/6;
  float spd = targetSpeed - spdDelta*abs(float(linePositon));
  return spd;
}

void controlMotorFromSensors(float targetSpeed, float deacceleratedSpeed=10){
  int linePos = getLinePos();
  float sensorPIDValue = getPIDsensorValue(linePos);
  float newTargetSpeed[2];
  delayAccelerationSpeed(targetSpeed, newTargetSpeed, deacceleratedSpeed);
  float rightSpeed = newTargetSpeed[RIGHT] - sensorPIDValue;
  float leftSpeed = newTargetSpeed[LEFT] + sensorPIDValue;
  controlEachMotorWithPID(rightSpeed, leftSpeed);
}

void showSpeed(){
  Serial1.print("Speed: ");
  Serial1.print(currentSpeed[RIGHT]);
  Serial1.print(", ");
  Serial1.print(80);
  Serial1.print(", ");
  Serial1.println(20);
}

void showLinePos(){
  Serial.println(getLinePos());
}

int initialMicros;
int loopCount = 0;
void showLoopSpeed(int loopNum){
  if(loopCount==0){
    initialMicros = micros();
  }
  loopCount++;
  
  if(loopCount==loopNum){
    Serial.print("Loop Speed ( ");
    Serial.print(loopNum);
    Serial.print(" iters): ");
    Serial.print((micros()-initialMicros)/loopNum);
    Serial.println("us");
    loopCount = 0;
    showSpeed();
  }
}


void initAll(){
  //global variables initialize
  currentSpeed[RIGHT] = 0;
  currentSpeed[LEFT] = 0;
  sensorErrorPrev = 0;
  sensorErrorSum = 0;
  spdErrorPrev[RIGHT] = 0;
  spdErrorPrev[LEFT] = 0;
  spdErrorSum[RIGHT] = 0;
  spdErrorSum[LEFT] = 0;
  initialMicros = 0;
  loopCount = 0;
  
  //Motors init
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  analogWrite(MR1, 0);
  analogWrite(MR2, 0);
  analogWrite(ML1, 0);
  analogWrite(ML2, 0);

  //encoder init
  pinMode(ENCODER_L, INPUT);
  pinMode(ENCODER_R, INPUT);

  //multiplexer input pins init
  pinMode(SENSOR_PIN, INPUT);
  
  //set the hardware
  gearRatio = 18.8;
  wheelDiameter = 5.6;
  distancePerMotorRotate = wheelDiameter*PI/gearRatio;
}

void Xbee(){
  if(Serial1.available()){
    char c = Serial1.read();
    switch (c){
      case 's':
      initAll();
      isActive = false;
      break;

      case 'b':
      initAll();
      isActive = true;
      break;
      default: break;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  initAll();
}

void loop() {
  Xbee();
  if (isActive){     
    getSpeedWithTime(currentSpeed);
    controlMotorFromSensors(100);
  }else{
    controlEachMotorWithPID(0,0);
  }
}