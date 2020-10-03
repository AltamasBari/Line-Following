int b=0, w=1,c=0,c1=0,c2=0;
#define LF1 4
#define LB1 5
#define RF1 2
#define RB1 3
#define leftIR 13
#define rightIR 10

void setup() 
{
  Serial.begin(9600);
  pinMode(LF1,OUTPUT);
  pinMode(LB1,OUTPUT);
  pinMode(RF1,OUTPUT);
  pinMode(RB1,OUTPUT);
  pinMode(13,INPUT);
  pinMode(10,INPUT);
  pinMode(22,INPUT);
  pinMode(24,INPUT);
  pinMode(26,INPUT);
  pinMode(28,INPUT);
  pinMode(30,INPUT);
  pinMode(32,INPUT);
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(12,OUTPUT);
  /*digitalWrite(8,HIGH);
  digitalWrite(11,HIGH);
  digitalWrite(9,LOW);
  digitalWrite(12,LOW);*/
}

void loop() 
{
  /*digitalWrite(8,HIGH);
  digitalWrite(11,HIGH);
  digitalWrite(9,LOW);
  digitalWrite(12,LOW);*/
  
  
  int d1,d2,d3,d4,d5,d6,sl,sr,r,i;
  d1=digitalRead(22);
  d2=digitalRead(24);
  d3=digitalRead(26);
  d4=digitalRead(28);
  d5=digitalRead(30);
  d6=digitalRead(32);
  sl=digitalRead(13);
  sl=digitalRead(10);
  Serial.print("\n");
  Serial.print(d1);
  Serial.print(d2);
  Serial.print(d3);
  Serial.print(d4);
  Serial.print(d5);
  Serial.print(d6);
    if(d1==w && d2==w && d3==b && d4==b && d5==w && d6==w)
    forward();
    else if(d1==w && d2==w && d3==b && d4==b && d5==b && d6==w)
     sright();
    else if(d1==w && d2==b && d3==b && d4==w && d5==w && d6==w)
     sleft();
    else if(d1==w && d2==w && d3==w && d4==w && d5==b && d6==b)
     sright();
    else if(d1==b && d2==b && d3==w && d4==w && d5==w && d6==w)
     sleft();
    else if(d1==w && d2==w && d3==b && d4==b && d5==b && d6==b)
    hright();
    else if(d1==w && d2==b && d3==b && d4==b && d5==b && d6==b)
    hright();
    else if(d1==w && d2==b && d3==b && d4==w && d5==w && d6==w)
    sleft();
    else if(d1==b && d2==b && d3==w && d4==w && d5==w && d6==w)
    hleft();
    //black-white-black
    else if(d1==b && d2==b && d3==w && d4==w && d5==b && d6==b)
    hleft();
    else if(d1==b && d2==b && d3==b && d4==w && d5==w && d6==1)
    sright();
    else if(d1==b && d2==b && d3==b && d4==b && d5==w && d6==w)
    sright();
    else if(d1==b && d2==w && d3==w && d4==b && d5==b && d6==b)
    sleft();
    else if(d1==w && d2==w && d3==b && d4==b && d5==b && d6==b)
    sleft();
    else if(d1==w && d2==w && d3==w && d4==w && d5==w && d6==w)
    hright();
    else if(d1==b && d2==w && d3==w && d4==b && d5==b && d6==b)
    sleft();
    else if(d1==b && d2==b && d3==w && d4==w && d5==w && d6==b)
    forward();
    else if(d1==b && d2==b && d3==w && d4==b && d5==w && d6==w)
    forward();
    else if(d1==b && d2==b && d3==w && d4==b && d5==b && d6==b)
    sleft();
    else if(d1==b && d2==b && d3==w && d4==w && d5==w && d6==w)
    hright();
    else if(d1==w && d2==w && d3==w && d4==w && d5==b && d6==b)
    hleft();
    
    if(digitalRead(13)==w && digitalRead(10)==b)
    c1=1;
    while(c1==1)
    {
      r=digitalRead(10);
      if(digitalRead(10)!=r)
      c2++;
      if(digitalRead(13)==1 && digitalRead(10)==0)
      c1=2;
    }
    if(c1==2)
    {
     
     for(i=0;i<c2;i++)
     { 
      digitalWrite(A0,HIGH);
      delay(1000);
      digitalWrite(A0,LOW);
     }
     c1=3;
    }
    
 }

void forward()
{
  analogWrite(LF1,140);
  analogWrite(LB1,0);
  analogWrite(RF1,140);
  analogWrite(RB1,0);
  
  analogWrite(A0,0);
  analogWrite(A1,0);
}
void pause()
{
  analogWrite(LF1,0);
  analogWrite(LB1,0);
  analogWrite(RF1,0);
  analogWrite(RB1,0);
}
void sright()
{
  analogWrite(LF1,120);
  analogWrite(LB1,0);
  analogWrite(RF1,0);
  analogWrite(RB1,60);
}
void sleft()
{                                     
  analogWrite(LF1,0);
  analogWrite(LB1,30);
  analogWrite(RF1,120);
  analogWrite(RB1,0);
}
void hleft()
{
  analogWrite(LF1,0);
  analogWrite(LB1,60);
  analogWrite(RF1,150);
  analogWrite(RB1,0);
}
void hright()
{
  analogWrite(LF1,0);
  analogWrite(LB1,150);
  analogWrite(RF1,60);
  analogWrite(RB1,0);
}
