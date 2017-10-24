#include <Wire.h>
#include <Servo.h>
#include <Encoder.h>

#define SLAVE_ADDRESS 0x04

Servo left_motor;
Servo right_motor;

double left_measurement;
double right_measurement;
double left_setpoint = 15000;
double right_setpoint = 15000;
double left_output;
double right_output;
double left_sum = 0;
double right_sum = 0;
double left_prev_error = 0;
double right_prev_error = 0;
double kP = 0.2f, kI = 0.0001f, kD = 0.001f, dt = 0.001;


struct Wheel {
  Encoder encoder_;
  Servo motor_;
  double speed_;
  double output_;
  double input_;
  double wheel_radius_;
  void calculate();
};

Encoder left_encoder(18, 19);
Encoder right_encoder(20, 21);

void setup() {
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  left_motor.attach(2);

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  Serial.begin(9600);           
}

void loop() {
  left_measurement = left_encoder.read();
  //right_measurement = right_encoder.read();

  Serial.println(left_measurement);
  //Serial.println(right_measurement);

  float left_error = left_setpoint - left_measurement;
  float left_dError = (left_error - left_prev_error) / dt;
  left_prev_error = left_error;
  left_sum += left_error * dt;
  float left_output = kP * left_error + kD * left_dError + kI * left_sum;
  left_output /= 500;

  if(left_output > 1) {
    left_output = 1;
  }
  else if(left_output < -1) {
    left_output = -1;
  }
  
  //Serial.println(left_output);
  //Serial.println(right_output);

  Serial.println(transformToPWM(-left_output));


  // Range from 968 to 2032 (1500 +/- 532)
  left_motor.writeMicroseconds((int) transformToPWM(-left_output));
  
  delay(1);
}


void receiveData(int byteCount){

  
  if(byteCount == 1){
    Wire.read();
    Wire.write(0);
    Wire.write(0);
    return;
  }

  if(byteCount != 2) return;

  if(Wire.available()) {
    int left_read = Wire.read();
    left_setpoint = map(left_read, 0, 255 , 2000, 1000);     

    Serial.print(left_setpoint); 
  }
  Serial.print(", "); 
  if(Wire.available()) {
    int right_read = Wire.read();
    right_setpoint = map(right_read, 0, 255 , 1000, 2000);     

    Serial.println(right_setpoint); 
  }

}

void sendData(){
  Wire.write(0);
}

// Transforms a -1 to 1 input into a Talon PWM value
int transformToPWM(float input) {
  // Range from 968 to 2032 (1500 +/- 532)
  return (int) (input * 532 + 1500);
}

