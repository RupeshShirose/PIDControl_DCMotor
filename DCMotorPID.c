#define ENCA 2 //Encode pulse A
#define ENCB 3 //Encoder pulse B
#define R_PWM 6 //PWM R for motor driver- clockwise rotation
#define L_PWM 5 //PWM L for motor driver- counterclockwise rotaion
#define PWM 9 //PWM pulses to the driver

float pos=0;      //intitial position
long prevT=0;     //t-1 time
float eprev=0;    //error at t-1
float eintegral=0; //error iin intergral

void setup(){
  Serial.begin(9600);
  pinMode(ENCA, INPUT); //encoder pulse A
  pinMode(ENCB, INPUT); //encoder pulse B
  //Interrupt to read encoder, remember to connect atleast one pin for encoder to interrrupt pin of UNO (D3,D2)
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING); 
  pinMode(R_PWM, OUTPUT); //CW
  pinMode(L_PWM, OUTPUT); //CCW
  pinMode(PWM,OUTPUT); //Two pins at motor driver shorted(R_EN ,L_EN) and given PWM, never give high signal to both at a time 
  //pinMode(R_EN, OUTPUT);
  //pinMode(L_EN,OUTPUT); 
  //digitalWrite(R_EN, HIGH);
  //digitalWrite(L_EN,HIGH);
}



void loop()
{
  //Set target position / rotation
  //int rot =10;
  //int target=500*rot;
  int target=1000*cos(prevT/1e6);
  
  //PID constants for tuning
  float kp=1;
  float kd=0.020;
  float ki=0.001;

  //time difference
  long currT=micros();//Current time in microsecond

  float deltaT=((float)(currT-prevT))/1.0e6; //caculating time in seconds
  prevT=currT;

  //error calcluation
  int e = target-pos;
  
  //derivative
  float dedt= (e-eprev)/(deltaT);

  //integral
  eintegral=eintegral+e*deltaT;

  //Control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  //motor power
  float pwr=fabs(u); //absolute value
  if(pwr>255)
  {
    pwr=255;
  }
  //if(pwr<0)
 // {
   // pwr=0;
 // }

  //Direction
  int dir=1;
  if(u<0)
  {
    dir=-1;
  }

  //Signal the motor
  setMotor(dir,pwr,PWM,R_PWM, L_PWM);

  //To complete closed loop we store previous error
  eprev = e;


  //To print target and position to serial
  Serial.println(target);
  Serial.println(" ");
  Serial.println(pos);
  Serial.println(" ");
  
 //setMotor(1,15,PWM,R_PWM,L_PWM);
 //delay(200);
 //Serial.println(pos);
 //setMotor(-1,15,PWM,R_PWM,L_PWM);
 //delay(200);
 //Serial.println(pos); 
 //setMotor(1,15,PWM,R_PWM,L_PWM);
 //delay(200);
 //Serial.println(pos); 
 
 
  //Serial.println(pos);
 
  //int a=digitalRead(ENCA);
  //int b=digitalRead(ENCB);
  //Serial.print(a*5);
  //Serial.print(" ");
  //Serial.print(b*5);
  //Serial.println();
  
}


void setMotor(int dir, int pwmVal, int pwm, int r_pwm, int l_pwm)
{
  analogWrite(pwm, pwmVal);
  if(dir==1) //Clockwise
  {
    digitalWrite(r_pwm, HIGH);
    digitalWrite(l_pwm, LOW);
  }
  else if(dir==-1) //Counterclockwise
  {
    digitalWrite(r_pwm,LOW);
    digitalWrite(l_pwm,HIGH);
  }
  else
  {
    digitalWrite(r_pwm, LOW);
    digitalWrite(l_pwm,LOW);
  }
}



void readEncoder()
{
  float b = digitalRead(ENCB);
  if(b>0){
    pos++;
  }
  else{
    pos--;
  }
  
}
