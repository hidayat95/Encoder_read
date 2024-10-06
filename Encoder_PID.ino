
//Motor 1 enc
const int chA_R = 2;  //2
const int chB_R = 4;  //4

// Motor 2 enc
const int chA_2 = 3;
const int chB_2 = 5;

//Motor Driver
const int M1_dir = 7;
const int M1_dir2 = 8;
const int M1_pwm = 11;

const int M2_dir = 9;
const int M2_dir2 = 10;
const int M2_pwm = 6;

float pulsesChanged_R = 0;
float pulses_R; 
float pulsesChanged_2 = 0;
float pulses_2;


//Variable
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
int pwmOut = 0;
int task =0;


//PID constants
double kp = 0.90;   double ki = 0.01;   double kd = 0.00;
double PID_p = 0;    double PID_i = 0;    double PID_d = 0;

//Common Variable
float time;
int period = 1000; //200ms


//inv kinematics
const double a1 = 12.5;  // length of link a1 in cm
const double a2 = 12.5; // length of link a2 in cm
const double a3 = 0;    // length of link a3 in cm
const double a4 = 7;    // length of link a4 in cm

double theta_1;
double theta_2;
  

void setup() {

  pinMode(M1_dir, OUTPUT);
  pinMode(M1_dir2, OUTPUT);
  pinMode(M1_pwm, OUTPUT);
  pinMode(M2_dir, OUTPUT);
  pinMode(M2_dir2, OUTPUT);
  pinMode(M2_pwm, OUTPUT);
  
  pinMode(chA_R, INPUT_PULLUP);
  pinMode(chB_R, INPUT_PULLUP);
  pinMode(chA_2, INPUT_PULLUP);
  pinMode(chB_2, INPUT_PULLUP);
  

  Serial.begin(115200);
  
  attachInterrupt(digitalPinToInterrupt(chA_R),encoder_readR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(chA_2),encoder_read2,CHANGE);

 Time = millis();  

}

void loop() {

    inv_kine(4,10);      // put coordiantes here (x,y)
    
    pid_function_1(theta_1);
    pid_function_2(theta_2);
  
}


///// inv kine/////
void inv_kine(double  x, double  y){
  // Desired Position of End effector


  // Equations for Inverse kinematics
    double r1 = sqrt(pow(x,2)+pow(y,2));  // eqn 1
    double phi_1 = acos((sq(a4)-sq(a2)-sq(r1))/(-2*a2*r1));  // eqn 2
    double phi_2 = atan2(y, x);  // eqn 3
    theta_1 = (phi_2-phi_1)*57.2958;  // eqn 4 converted to degrees

    double phi_3 = acos((sq(r1)-sq(a2)-sq(a4))/(-2*a2*a4));
    theta_2 = 180-(phi_3*57.2958);

    Serial.print("theta one: ");
    Serial.print(theta_1);
    Serial.print("   theta two: ");
    Serial.println(theta_2);
}


void encoder_readR() {                                     //Interrupt function to read the x2 pulses of the encoder.
  if ( digitalRead(chB_R) == 0 ) {
    if ( digitalRead(chA_R) == 0 ) {
      // A fell, B is low
      pulses_R--; // Moving forward
    } else {
      // A rose, B is high
      pulses_R++; // Moving reverse
    }
  } else {
    if ( digitalRead(chA_R) == 0 ) {
      pulses_R++; // Moving reverse
    } else {
      // A rose, B is low
      pulses_R--; // Moving forward
    }
  }
  pulsesChanged_R = 1;
}

///////////// MOTOR 2 ENC ////////////
void encoder_read2() {                                     //Interrupt function to read the x2 pulses of the encoder.
  if ( digitalRead(chB_2) == 0 ) {
    if ( digitalRead(chA_2) == 0 ) {
      // A fell, B is low
      pulses_2--; // Moving forward
    } else {
      // A rose, B is high
      pulses_2++; // Moving reverse
    }
  } else {
    if ( digitalRead(chA_2) == 0 ) {
      pulses_2++; // Moving reverse
    } else {
      // A rose, B is low
      pulses_2--; // Moving forward
    }
  }
  pulsesChanged_2 = 1;
}


//////////////// PID FUNC. /////////////////
void pid_function_1(float set_angle)
{
  float pulse_per_rotation = 3780;
  float angle_pulse = (set_angle*(pulse_per_rotation/360));
  
  PID_error = angle_pulse - pulses_R;
    PID_p = kp * PID_error;
      if(-1 < PID_error <1){
        PID_i = PID_i + (ki * PID_error);
      }
      else{
        PID_i = 0;
      }
    timePrev = Time; 
    Time = millis();
    elapsedTime = (Time - timePrev) / 1000; 
    PID_d = kd*((PID_error - previous_error)/elapsedTime);
    PID_value = PID_p + PID_i + PID_d;

    if (abs(PID_value*1.7) > 255){
    pwmOut = 255;
   }
   else if (abs(PID_value) < 0){
    pwmOut = 0;
   }
   else{
    pwmOut = abs(PID_value*1.7);
   }

   if (PID_value > 0){
    analogWrite(M1_pwm, pwmOut);
    digitalWrite(M1_dir, LOW);
    digitalWrite(M1_dir2, HIGH);
   }
   else if (PID_value < 0){
    analogWrite(M1_pwm, pwmOut);
    digitalWrite(M1_dir, HIGH);
    digitalWrite(M1_dir2, LOW);
   }
  
//  Serial.print(pulses_R);
//  Serial.print(" pulses    ");
//  Serial.print(PID_error);
//  Serial.print("  ");
//  Serial.print(task);
//  Serial.print("  ");
//  Serial.print(pwmOut);
//  Serial.print(" pwm_out   ");
//  Serial.println(PID_value);
}



void pid_function_2(float set_angle)
{
  float pulse_per_rotation = 3780;
  float angle_pulse = (set_angle*(pulse_per_rotation/360));
  
  PID_error = angle_pulse - pulses_2;
    PID_p = kp * PID_error;
      if(-1 < PID_error <1){
        PID_i = PID_i + (ki * PID_error);
      }
      else{
        PID_i = 0;
      }
    timePrev = Time; 
    Time = millis();
    elapsedTime = (Time - timePrev) / 1000; 
    PID_d = kd*((PID_error - previous_error)/elapsedTime);
    PID_value = PID_p + PID_i + PID_d;

    if (abs(PID_value) > 255){
    pwmOut = 255;
   }
   else if (abs(PID_value) < 0){
    pwmOut = 0;
   }
   else{
    pwmOut = abs(PID_value*1.7);
   }

   if (PID_value > 0){
    analogWrite(M2_pwm, pwmOut);
    digitalWrite(M2_dir, LOW);
    digitalWrite(M2_dir2, HIGH);
   }
   else if (PID_value < 0){
    analogWrite(M2_pwm, pwmOut);
    digitalWrite(M2_dir, HIGH);
    digitalWrite(M2_dir2, LOW);
   }
  

//  Serial.print(pulses_2);
//  Serial.print(" pulses    ");
//  Serial.print(PID_error);
//  Serial.print("  ");
//  Serial.print(elapsedTime);
//  Serial.print("  ");
//  Serial.print(pwmOut);
//  Serial.print(" pwm_out   ");
//  Serial.println(PID_value);
}
