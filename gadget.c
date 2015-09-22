#include <phys253.h>        //   ***** from 253 template file
#include <LiquidCrystal.h>  //   ***** from 253 template file
#include <servo253.h>       //   ***** from 253 template file 

#define LOOP2 0
#define LOOP8 1 //for readability
#define TRACKIN 1
#define TRACKOUT 0
#define CENTER 90
#define RFTHRESH 179
#define ZZTIME 150
#define STOP_THRESH 900
#define STOP_TIME 1111

int MIN_ANGLE=CENTER-39;
int MAX_ANGLE=CENTER+39;

int menu=0;

int s_ref8 = 11;
int s_ref = 11;
//int ref_angle[8] = {0, 12, 24, 36, 48, 60, 72, 84};
int var_angle;
int ref_angle[8] = {0, 3, 5, 9, 12, 16, 21, 37};
int angle=CENTER;
long f_angle=CENTER;//this goes into SERVO
long f_angle0=CENTER;//the old f_angle;
int p_angle[15];//is assigned later from ref_angle

long dt; long t0; long t1; long kd=10; long der_angle=0;

boolean loop28 = LOOP8;//TRUE IF RUNNING LOOP8, FALSE IF RUNNING LOOP2. starts with LOOP8 by default
boolean loopmotor = 1;

int motorSpeed = 1023;
const int BRAKE_CONSTANT=200;//milliseconds to brake!
const int ACCEL_CONSTANT=210 ;//milliseconds to accelerate after braking! (note, the amount of time it accelerates is this minus the BRAKE_CONSTANT)
const int motorSpeedCurve[15]={400,480,560,640,720,800,880,1023,880,800,720,640,560,480,400};
int curveFactor = 0;//MUST BE BETWEEN 0 and 14 (inclusive).
//const int curveSounds[16]={278,294,312,330,350,371,393,416,441,467,495,524,556,589,624,661};

const int numIterations=3300;//The higher this is, the smoother the steering.
long tStart;//For testing how frequently the robot scans the tape.
long tRun;//time counter for motor speed control.

boolean inner=TRACKOUT;//Is it on the inner track?
boolean lanechange=0;    //Do we want to change lanes?
int lanestage=0;
long lanetime;
boolean lanet=0;
boolean laneon;//have we left original lane
int lanelastsen;//last sensor
int delaytime=133;//for lanechanging

long zztime;
boolean zztimer=0;

long i_angle=0;
long ki=0;
boolean iord;//buttons/knob set ki or kd: I true D false (T or F = I or D)
int knob70;//previous knob7 value; only print knob(7)!=knob70


void setup(){
  portMode(0, INPUT); portMode(1, INPUT);      //   ***** from 253 template file
  RCServo0.attach(RCServo0Output);RCServo1.attach(RCServo1Output);RCServo2.attach(RCServo2Output);
  RCServo0.write(CENTER);
  
  while(!startbutton()&&!stopbutton()){
    LCD.clear(); LCD.home(); 
    LCD.print("STOP br START se");
    LCD.setCursor(0,1);
    for(int i=8;i<=15;i++){
      if(digitalRead(i)) LCD.print("1");
      else               LCD.print("0");
    }
    delay(200);
  }
  while(startbutton()||stopbutton()){delay(100);}
  MIN_ANGLE=CENTER-39;
  MAX_ANGLE=CENTER+39;
  while(!startbutton()){//WE KEEP THIS INNER//OUTER SETUP
    LCD.clear(); LCD.home();
    if(inner==TRACKOUT) LCD.print("OUTER LANE?");
    else LCD.print("INNER LANE?");
    if(stopbutton()){
      while(stopbutton()){delay(100);}
      if(inner==TRACKIN){
        inner=TRACKOUT;
        digitalWrite(31,HIGH);
      }
      else{
        inner=TRACKIN;
        digitalWrite(33,HIGH);
      }
    }
    delay(300);
  }
  while(startbutton()){delay(100);}
  while(!startbutton()){
    if(stopbutton()){
      while(stopbutton()){delay(100);}
      if(menu!=2) menu++;
      else menu=0;
    }
    LCD.clear(); LCD.home();
    LCD.print("Which angles?");
    LCD.setCursor(0,1);
    if(menu==0) LCD.print("PRESET IN CODE->");
    else if(menu==1) LCD.print("CUSTOM ANGLES-->");
    else LCD.print("PROPORTIONAL--->");
    delay(200);
  }
  while(startbutton()){delay(100);}
  LCD.clear(); LCD.home(); 
  if(menu==0){}
  else if (menu==1){
    for(int i=0;i<=7;i++){//set the angles using knob 6.
      while(!startbutton()){
        if(stopbutton()){//exit
          while(stopbutton()) delay(100);
          i=8;
          break;
        }
        LCD.clear(); LCD.home(); 
        LCD.print("Angle #"); LCD.print(i); LCD.print(" is ");
        var_angle=knob(6)*9/100;
        LCD.print(var_angle);
        LCD.setCursor(0,1);
        LCD.print("(turn knob 6)");
        delay(100);
      }
      ref_angle[i]=var_angle;
      while(startbutton()){delay(100);}//so you don't skip
    }
  }else{
    ref_angle[0]=0;
    while(!startbutton()){
        LCD.clear(); LCD.home(); 
        LCD.print("INTERVAL: ");
        var_angle=knob(6)*9/100;
        LCD.print(var_angle);
        LCD.setCursor(0,1);
        LCD.print("(turn knob 6)");
        delay(100);
    }
    for(int i=1;i<=7;i++){
      ref_angle[i]=var_angle*i;
    }
    while(startbutton()){delay(100);}
  }
  
  delay(100);
  
  //ASSIGN ANGLES FROM REFERENCE
  for(int i=0;i<=7;i++){
    p_angle[i+7]=ref_angle[i]+CENTER;
    p_angle[7-i]=-ref_angle[i]+CENTER;
  }

  LCD.clear(); LCD.home();
  LCD.print("ANGLES ");//Prints out all angles on the screen. 
  LCD.print(p_angle[0]); LCD.print(" "); LCD.print(p_angle[1]); LCD.print(" ");
  LCD.print(p_angle[2]); LCD.setCursor(0,1); LCD.print(p_angle[3]); LCD.print(" ");
  LCD.print(p_angle[4]); LCD.print(" "); LCD.print(p_angle[5]); LCD.print(" ");
  LCD.print(p_angle[6]); LCD.print(" "); LCD.print(p_angle[7]);
  
  while(startbutton()){delay(100);}
  while(!startbutton()){delay(100);}
  while(startbutton()){delay(100);}
  
  LCD.clear(); LCD.home();
  LCD.setCursor(0,1);
  LCD.print("Sensor Check");
  while(!startbutton()){
    LCD.home();
    for(int i=8;i<=15;i++){
      if(digitalRead(i)) LCD.print("1");
      else               LCD.print("0");
    }
    delay(50);
  }
  LCD.clear(); LCD.home();
  LCD.print("STARTING IN:");
  LCD.setCursor(0,1); LCD.print("3"); delay(300);
  LCD.setCursor(0,1); LCD.print("2"); delay(300);
  LCD.setCursor(0,1); LCD.print("1"); delay(300);
  LCD.clear();LCD.home();
  tRun=millis();
}

void loop(){
  for(int j=1;j<=numIterations;j++){
    tStart=micros();
    if(lanechange==0){
      if(loop28){                     //IN LOOP8 MODE
        //THINGS THAT DON"T WORK ARE COMMENTED OUT!!!
        /*if(s_ref8==15) s_ref8=8;
        else s_ref8++;
        if(digitalRead(s_ref8)){
          //ACCIDENTAL LANE CHANGE
          
          //if(last sensor WAS on left, it WAS on the inner track, and it is NOW on right, it becomes outer)
          //or(last sensor WAS right, OUTER track, now on LEFT. it must be on inner
          if(s_ref<=11&&inner==TRACKIN&&s_ref8>=12||s_ref>=12&&inner==TRACKOUT&&s_ref8<=11)
          inner=!inner;
          
          if(s_ref8==15) s_ref8=14;
          
          s_ref=s_ref8;
          
          loop28=LOOP2;               //If something is detected, loop on 2!!
          
        }*/
        if(s_ref==15) s_ref=8;
        else s_ref++;
        if(digitalRead(s_ref)){
          if(s_ref==15) s_ref=14;
          loop28=LOOP2;               //If something is detected, loop on 2!!
        }
      }else{                           //IN LOOP2 MODE
        if(digitalRead(s_ref)){
          zztime=0;
          if(digitalRead(s_ref+1)){   //If both relevant sensors are on
            curveFactor=2*(s_ref-8)+1;
            angle=p_angle[curveFactor];
          }else{                      //If 10, shift s_ref once leftwards.
            curveFactor=2*(s_ref-8);
            angle=p_angle[curveFactor];
            if(s_ref>10) s_ref--;
          }
        }else{
          if(digitalRead(s_ref+1)){   //If 01, shift s_ref once rightwards.
            curveFactor=2*(s_ref-8)+2;
            angle=p_angle[curveFactor];
            if(s_ref<12) s_ref++;
            zztime=0;
          }else{                      //If nothing is detected, loop on 8!!
            if(zztime==0){
              zztimer=millis();
              zztime=1;}
              else
              if(millis()-zztimer>ZZTIME){
                zztime=0;
                loop28=LOOP8;
                s_ref=8;// Start loop8 from the left, so that it doesn't get distracted by markers.
              }
          }
        }
        t1=micros();  //Use t1 so that you don't have to call micros() twice - faster and more precise
        dt=t1-t0; t0=t1; //DT is about 100
        der_angle=kd*(f_angle-f_angle0)/dt;
        //i_angle+=((long)(f_angle-CENTER)*dt*ki);//DT = 100, KI is from 0 to 1000
        //if(i_angle>10) i_angle=10; 
        //if(i_angle<-10) i_angle=-10;
        f_angle=angle-der_angle;//-i_angle;
        
        if(f_angle<MIN_ANGLE) f_angle=MIN_ANGLE; //0<=angle<=180
        if(f_angle>MAX_ANGLE) f_angle=MAX_ANGLE;
        if(f_angle!=f_angle0){
          RCServo0.write(f_angle);
          if(abs(f_angle-CENTER)>10&&abs(f_angle-f_angle0)>15){
            tRun=millis();
          }
        }
        f_angle0=f_angle;
      }
    }else{//LANE CHANGE PROCESS BEGINS
      if(lanestage==0){//Stage 0. Set the initial angle (left if on outer track, right if on inner track).
        if(inner==TRACKOUT) RCServo0.write(CENTER-35);
        else RCServo0.write(CENTER+35); 
        tRun=millis();//brake;
        curveFactor=0;
        lanestage=1;//NEXT STAGE
        laneon=1;//THIS IS TRUE, SINCE SENSORS ARE STILL ON THE ORIGINAL LANE
      }else if(lanestage==1){//1. Detect once we have left the initial lane, then set angle straight
        tRun=millis();//brake;
        if(laneon){//STILL ON INITIAL LANE?
          laneon=0;//ASSUME IT IS OFF THE LANE
          for(int i=8;i<=15;i++){
            if(digitalRead(i)){
              laneon=1;//ONE SENSOR DETECTED SOMETHING, THEREFORE WE HAVEN'T LEFT THE INITIAL LANE
              lanelastsen=i;
              break;//WE KNOW WE'RE ON INITIAL LANE, NO POINT IN SCANNING OTHER SENSORS
            }
          }
          //It's possible to go from 00100000->00000000->00010000
          //We assume we have left the lane if we have 00000000 and the last state was 00000001 or 10000000
          //However, we still might go from 00000001->00000000->00000001 (which happened before)
          if(!(lanelastsen==8||lanelastsen==15)) laneon=1;//while we may have 00000000, the last detection wasn't on one endpoint, so we must still be on the lane.
        }else{ 
          delay(delaytime);//Just to be safe; might not need this
          RCServo0.write(CENTER);//Go straight!
          lanestage=2;//NEXT STAGE
        }
      }else if(lanestage==2){//Stage 2. Detect once we have reached a track. //NEW THEN TURN
      //If we are going from OUTER to INNER, we should first detect something on one of the leftmost sensors (otherwise we failed).
        if(inner==TRACKOUT){//Going from outer to inner
          for(int i=8;i<=15;i++){
            if(digitalRead(i)){
              if(i<=11) RCServo0.write(CENTER+20);//Success! We detected a track on the left!
              else inner=TRACKIN;//Failure! We are back on original track! Treat it as though we were initially on inner track and have gone to outer track.
              lanestage=3;//NEXT STAGE
              break;
            }
          }
        }else{//same as above but opposite
          for(int i=8;i<=15;i++){
            if(digitalRead(i)){
              if(i>=12) RCServo0.write(CENTER-20);
              else inner=TRACKOUT;
              lanestage=3;
              break;
            }
          }
        }  
      }else if(lanestage==3){//Stage 3. Don't do anything until you see something on one of the four sensors on the side AWAY from the new track (Otherwise it won't turn properly).
        if(inner==TRACKOUT){
          for(int i=12;i<=15;i++){
            if(digitalRead(i)) lanestage=4;
          }
        }else{
          for(int i=8;i<=11;i++){        
            if(digitalRead(i)) lanestage=4;
          }
        }
      }else if(lanestage==4){//4. We did it! Well done
        inner=!inner;//The state of the lane has changed.
        lanestage=0;
        lanechange=0;
      }
    }//END OF LANE CHANGE PROCESS
    
    //MOTOR SPEED CONTROL
    if(millis()-tRun>ACCEL_CONSTANT){
      analogWrite(5,motorSpeed);
    }else if(millis()-tRun>BRAKE_CONSTANT){
      analogWrite(5,1023);//MAX POWER!!!
    }else{
      analogWrite(5,min(motorSpeedCurve[curveFactor],motorSpeed));
    }
    //ACTUAL LANE CHANGE DETECTOR
    if(lanet==0&&lanechange==0&&digitalRead(0)){
      lanetime=millis();
      lanet=1;
    }else if(lanet==1){
      if(!digitalRead(0)) lanet=0;
      else if(millis()-lanetime>RFTHRESH){
        lanechange=1;
        lanet=2;
      }
    }else if(lanet==2){
      if(!digitalRead(0)) lanet=0;
      else if(millis()-lanetime>STOP_THRESH){
        analogWrite(5,200);
        delay(STOP_TIME);
        analogWrite(5,1023);
        delay(ACCEL_CONSTANT);
        lanet=0;
      }
    }
  }//END OF FAST LOOP
  
  //These slow things below are done oonly once per many times that the above fast stuff loops!
  motorSpeed=knob(6);
  if(stopbutton()){
    iord=!iord;
    LCD.clear();LCD.home();
    if(iord){
      LCD.clear();LCD.home();
      LCD.print("KNOB 7: delaytime");
      LCD.setCursor(0,1);
      LCD.print("KNOB 6: POWER");
      while(stopbutton()){delay(300);}
    }else{
      LCD.clear();LCD.home();
      LCD.print("KNOB 7: KD");
      LCD.setCursor(0,1);
      LCD.print("KNOB 6: POWER");
      while(stopbutton()){delay(300);}
    }
  }
  if(startbutton()){
    while(startbutton()){delay(100);}
    if(iord){
      while(!startbutton()){
        LCD.clear();LCD.home();
        delaytime = knob(7);
        motorSpeed=knob(6);
        LCD.print("DELAY:"); LCD.print(delaytime);
        LCD.print(" PWR:"); LCD.print(motorSpeed);
        LCD.setCursor(0,1);
        for(int i=8;i<=15;i++){
          if(digitalRead(i)) LCD.print("1");
          else               LCD.print("0");
        }
        delay(200);
      }
    }else{
      while(!startbutton()){
        LCD.clear();LCD.home();
        kd = knob(7)/10;
        motorSpeed=knob(6);
        LCD.print("KD:"); LCD.print(kd);
        LCD.print(" PWR:"); LCD.print(motorSpeed);
        LCD.setCursor(0,1);
        for(int i=8;i<=15;i++){
          if(digitalRead(i)) LCD.print("1");
          else               LCD.print("0");
        }
        
        delay(200);
      }
    }
    while(startbutton()){delay(100);}
  }   
  
  //END OF CODE
  LCD.clear(); LCD.home(); 
  LCD.print("rngfnd:");LCD.print(digitalRead(0)?"1":"0");
  LCD.print(" pwr");LCD.print(motorSpeed);
  //LCD.print(lanechange?("LANE CHANGE"):(" NO  CHANGE"));
  LCD.setCursor(0,1);
  LCD.print(inner?("INNER"):("OUTER"));
  if(lanechange){
    LCD.print("..SWITCHING");
  }
}