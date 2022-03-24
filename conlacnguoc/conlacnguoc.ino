#define in1 4
#define in2 2
#define enA 9

#define in3 6
#define in4 8
#define enB 3
#define inPot 0


int speed = 0;
double kp = 70;
double ki = 0;
double kd = 0;
int PID = 0;
int error = 0;
int last_error = 0;
int anpha = 0;
int sp = 487;



void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
 pinMode(enA,OUTPUT);
 pinMode(in1,OUTPUT);
 pinMode(in2,OUTPUT);
 pinMode(in3,OUTPUT);
 pinMode(in4,OUTPUT);
 pinMode(enB,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  control();
}

int calcPID(int error)
{
  PID = (kp*error+ki*(error+last_error)+kd*(error-last_error));
  last_error = error;
  return constrain(PID, 0, 255);
}

int pot()
{
  unsigned long avg = 0;
  for( int i=0; i<500; i++)
  {
    unsigned long valPot = analogRead(inPot);
    avg += valPot;
  }
  avg = avg/500;
  //Serial.println(avg);
  return avg;
}

void tien(int i)
{
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  
  analogWrite(enB,i);
  analogWrite(enA,i);
  
}

void lui(int i)
{
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);  
  
  analogWrite(enB,i);
  analogWrite(enA,i);
  
}

void dungyen()
{
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);  
  
  analogWrite(enA,0);
  analogWrite(enB,0);
}

void control()
{
  anpha = pot();
  if(anpha <=440 || anpha >= 540)
  {
    dungyen();
  }
  else
  {
    if(anpha < sp)
    {
      speed = calcPID((sp - anpha));
      lui(speed);
    }
    if(anpha > sp)
    {
      speed = calcPID((anpha - sp));
      tien(speed);
    }
    Serial.println(speed);
  }
}
