float Kp = 1;
float Ki = 1;
float Kd = 1;

//setup speed of the motors
#define ENAspeed 200
#define ENBspeed 200

#define ENB 5 // Enable pin PWM of the Left motor
#define IN4 6 // pin of left motor
#define IN3 7 // pin of left motor
#define IN2 8 // pin of right motor
#define IN1 9 // pin of right motor
#define ENA 10 // Enable pin PWM of the Right motor

int Sensor[] = {A1, A2, A3, A4, A5};
float P = 0;
float I = 0;
float D = 0;
int Error = 0;

int Previous_Error, Previous_I;
int PID_value;
int Left_Speed, Right_Speed;

void setup(){
    
     pinMode(9, OUTPUT);
     pinMode(8, OUTPUT);
     pinMode(7, OUTPUT);
     pinMode(6, OUTPUT);
     pinMode(5, OUTPUT);
     pinMode(10, OUTPUT);
     
     pinMode(A1, INPUT);
     pinMode(A2, INPUT);
     pinMode(A3, INPUT);
     pinMode(A4, INPUT);
     pinMode(A5, INPUT);
     
     digitalWrite(IN1, HIGH);
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, HIGH);
     digitalWrite(IN4, LOW);
     
     analogWrite(ENB, Left_Speed);
     analogWrite(ENA, Right_Speed);

     for (int i = 1; i <= 5; i++)
      {Sensor[i];}
     // reading the sensor if HIGH (Black line) or LOW (White line) line
     Sensor[1] = digitalRead(A1);
     Sensor[2] = digitalRead(A2);
     Sensor[3] = digitalRead(A3);
     Sensor[4] = digitalRead(A4);
     Sensor[5] = digitalRead(A5);
}

    void loop() {
    //use analogwrite to run motor at adjusted speed
     CalculateError();
     Calculate_PID();
     motor_control();
}
                                                                                                                                                                 
  void CalculateError(){
  if ((Sensor[1] == 1) && (Sensor[2] == 0) && (Sensor[3] == 0) && (Sensor[4] == 0) && (Sensor[5] == 0))  
  {Error = 4;}
  else if ((Sensor[1] == 1) && (Sensor[2] == 1) && (Sensor[3] == 0) && (Sensor[4] == 0) && (Sensor[5] == 0)) 
  {Error = 3;}
  else if ((Sensor[1] == 0) && (Sensor[2] == 1) && (Sensor[3] == 0) && (Sensor[4] == 0) && (Sensor[5] == 0)) 
  {Error = 2;}
  else if ((Sensor[1] == 0) && (Sensor[2] == 1) && (Sensor[3] == 1) && (Sensor[4] == 0) && (Sensor[5] == 0)) 
  {Error = 1;}
  else if ((Sensor[1] == 0) && (Sensor[2] == 0) && (Sensor[3] == 1) && (Sensor[4] == 0) && (Sensor[5] == 0)) 
  {Error = 0;}
  else if ((Sensor[1] == 0) && (Sensor[2] == 0) && (Sensor[3] == 1) && (Sensor[4] == 1) && (Sensor[5] == 0)) 
  {Error = -1;}
  else if ((Sensor[1] == 0) && (Sensor[2] == 0) && (Sensor[3] == 0) && (Sensor[4] == 1) && (Sensor[5] == 0)) 
  {Error = -2;}
  else if ((Sensor[1] == 0) && (Sensor[2] == 0) && (Sensor[3] == 0) && (Sensor[4] == 1) && (Sensor[5] == 1)) 
  {Error = -3;}  
  else if ((Sensor[1] == 0) && (Sensor[2] == 0) && (Sensor[3] == 0) && (Sensor[4] == 0) && (Sensor[5] == 1)) 
  {Error = -4;}
  //Special case
  else if ((Sensor[1] == 0) && (Sensor[2] == 0) && (Sensor[3] == 0) && (Sensor[4] == 0) && (Sensor[5] == 0))
  {if (Previous_Error = -4) {Error = -5;}
  else if (Previous_Error = 4) {Error = 5;}} 
  else if ((Sensor[1] == 1) && (Sensor[2] == 1) && (Sensor[3] == 1) && (Sensor[4] == 1) && (Sensor[5] == 1))
  {Error = 0;}
}

  void Calculate_PID() { 
    P = Error;
    I += Error;
    //I = I + Previous_I;
    D = Error - Previous_Error;

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);

    I += Error;
    Previous_I = I;
    Previous_Error = Error;
}

  void motor_control() {
     
     //define new speed limitation for the engine
     Left_Speed  = constrain (Left_Speed, 0, 255);
     Right_Speed = constrain(Right_Speed, 0, 255);
     
     //calculating the effective motor speed:
          if ((-5 < Error) and (Error < 0)){
              Left_Speed  = ENBspeed - PID_value;
              Right_Speed = ENAspeed + PID_value;
          }

          if ((0 < Error) and (Error < 5)){
              Left_Speed  = ENBspeed + PID_value;
              Right_Speed = ENAspeed - PID_value;
          }

          if (Error == 0){
              Left_Speed  = ENBspeed;
              Right_Speed = ENAspeed;
          }
     }
