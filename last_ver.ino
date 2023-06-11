#include <POP32.h>
#include <POP32_Pixy2.h>
POP32_Pixy2 pixy;
#define degToRad 0.0175f
#define sin30 sin(30.f * degToRad)
#define cos30 cos(30.f * degToRad)
// rotate controller
#define rot_Kp 0.35
#define rot_Ki 0.0
#define rot_Kd 0.2
#define idleSpd 30  // speed when not discovered ball
#define rotErrorGap 12
float rot_error, rot_pError, rot_i, rot_d, rot_w;
int ballPosX;
// fling controller
#define fli_Kp1 2.0
#define fli_Kp2 2.0
#define fli_Ki 0.0
#define fli_Kd 1
#define flingErrorGap 15
float ballFli = 140;
float fli_error, fli_pError, fli_i, fli_d, fli_spd;
int ballPosY;
int goalPosX, goalPosY, goalWidth;
int x, width;// getGoal func
// align controller
#define ali_Kp 4
#define ali_Kd 0.1
#define alignErrorGap 3
float ali_error, ali_pError, ali_d, ali_vec, vecCurve, radCurve;
bool discoveState = true;
bool goalDiscoveState = false;
bool findGoalState = true;
int goalNum;
int goalFli = 90;
/* >> ball shooting <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
#define limPin PA0
#define reloadSpd 70
int timer = 0;
int ballX;
float thetaRad, vx, vy, spd1, spd2, spd3;
int myGoal, eneGoal;

void zeroYaw() {
  Serial1.begin(115200); delay(100);
  // Sets data rate to 115200 bps
  Serial1.write(0XA5); Serial1.write(0X54); delay(100);
  // pitch correction roll angle
  Serial1.write(0XA5); Serial1.write(0X55); delay(100);
  // zero degree heading
  Serial1.write(0XA5); Serial1.write(0X52); delay(100);
  // automatic mode
}

volatile float pvYaw;
volatile uint8_t rxCnt = 0, rxBuf[8];

bool getIMU() {//ฟังก์ชั่นรับมุม yaw ทวนค่าบวก ตามค่าลบ
  while (Serial1.available()) {
    rxBuf[rxCnt] = Serial1.read();
    if (rxCnt == 0 && rxBuf[0] != 0xAA) return;
    rxCnt++;
    if (rxCnt == 8) { // package is complete
      rxCnt = 0;
      if (rxBuf[0] == 0xAA && rxBuf[7] == 0x55) { // data package is correct
        pvYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.f;
        return true;
      }
    }
  }
  return false;
}
void wheel(int s1, int s2, int s3) {
  motor(1, s1);
  motor(2, s2);
  motor(3, s3);
}
void holonomic(float spd, float theta, float omega) {
  thetaRad = theta * degToRad;
  vx = spd * cos(thetaRad);
  vy = spd * sin(thetaRad);
  spd1 =   vy * cos30 - vx * sin30 + omega;
  spd2 = - vy * cos30 - vx * sin30 + omega;
  spd3 =   vx + omega;
  wheel(spd1, spd2, spd3);
}

/* heading ..............   */
#define head_Kp 2.3f
#define head_Ki 0.0f
#define head_Kd 0.5f

float head_error, head_pError, head_w , head_d, head_i;
void heading(float spd, float theta, float spYaw) {
  head_error = spYaw - pvYaw;
  head_i = head_i  + head_error;
  head_i = constrain(head_i , -180, 180);
  head_d =  head_error - head_pError;
  head_w = (head_error * head_Kp) + (head_i * head_Ki) + (head_d * head_Kd);
  head_w = constrain(head_w , -100, 100);
  holonomic(spd, theta, head_w);
  head_pError = head_error;
}

void shoot() {
  motor(4, reloadSpd);
  delay(150);
  motor(4, 0);
  delay(50);
}
void reload() {
  motor(4, reloadSpd);
  timer = 0;
  for (int i = 0; i < 2000; i++) {
    timer++;
    if (in(limPin)) break;
    delay(1);
  }/*
  if (timer == 2000) { // ถ้าก้านยิ่งติด
    motor(4, -reloadSpd); // เลื่อนก้านยิ่งไปข้างหน้า
    delay(500);     //ก่อน 0.5 วินาที
    motor(4, reloadSpd); 
    timer = 0;
    for (int i = 0; i < 2000; i++) {
      timer++;
      if (in(limPin)) break;
      delay(1);
    }
  }*/
  motor(4, 0);
}

bool getGoal(int goals) {//ฟังก์ชั่นคำนวณว่าโกลอยู่กลางจอหรือไม่
  if (pixy.updateBlocks())
  {
    if (goals == eneGoal && pixy.sigSize[eneGoal])
    {
      x = pixy.sigInfo[eneGoal][0].x;
      width = pixy.sigInfo[eneGoal][0].width;
    }
    else if (goals == myGoal && pixy.sigSize[myGoal])
    {
      x = pixy.sigInfo[myGoal][0].x;
      width = pixy.sigInfo[myGoal][0].width;
    }

    else return 0;
    // pixy.sigInfo[สีที่..(1-7)][ลำดับที่..(0-n)].x
//162=center
    if (width >= 162)
    {
      return 1;
    }
    else
    {
      if (x < 162)
      {
        if ((x + width / 2) > 164)
        {
          return 1;
        }
      }
      else
      {
        if ((x - width / 2) < 160)
        {
          return 1;
        }
      }
    }
  }
  return 0;
}

void goCatch(int x, int y, int spFli, int fli_Kp)
{
    rot_error = 162 - x;
    rot_i = rot_i + rot_error;
    rot_i = constrain(rot_i, -100, 100);
    rot_d = rot_error - rot_pError;
    rot_pError = rot_error;
    rot_w = (rot_error * rot_Kp) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
    rot_w = constrain(rot_w, -100, 100);
    
  if(abs(rot_error) < 30)
    {
      ballPosY = pixy.sigInfo[1][0].y;
      fli_error = spFli - y;
      fli_i = fli_i + fli_error;
      fli_i  = constrain(fli_i , -100, 100);
      fli_d = fli_error - fli_pError;
      fli_pError = fli_error;
      fli_spd = fli_error * fli_Kp + fli_i * fli_Ki + fli_d * fli_Kd;
      fli_spd = constrain(fli_spd , -100, 100);
    }
    else
    {
      fli_spd = 0;
    }
    holonomic(fli_spd, 90, rot_w);
}

bool whiteLine()
{
  if((analog(1)>330 || analog(3)>460 || analog(5)> 490) && !(pixy.sigSize[3] || pixy.sigSize[2]))
  {
    return true;
  }
  else return false;
}
void setup() {
  pixy.init();
  zeroYaw();
  oled.text(6, 0, "Press SW_B zeroYaw");
  oled.text(7, 0, "Press SW_A Run...");
  oled.show();
  int selectTeamState = 0;
  while (!selectTeamState)
  {//2 yellow
    //3 blue
    if(SW_A())
    {
      myGoal=2;
      eneGoal=3;
      oled.text(5,0,"My Goal = Yellow");
      oled.show();
      selectTeamState = 1;
    }
    else if(SW_B())
    {
      myGoal=3;
      eneGoal=2;
      oled.text(5,0,"My Goal = Blue");
      oled.show();
      selectTeamState = 1;
    }
  }
  while (!SW_OK ()) 
  {
    if (!SW_B ()) {
      zeroYaw();
    }
    getIMU();
    oled.text(0, 0, "Yaw = %f", pvYaw);
    oled.text(2,0,"Analog1 = %d",analog(1));
    oled.text(3,0,"Analog3 = %d",analog(3));
    oled.text(4,0,"Analog5 = %d",analog(5));
    oled.show();
  }
}
void loop()
{
  /*if (whiteLine()){
    while(pixy.updateBlocks()&&!(pixy.sigSize[eneGoal]||pixy.sigSize[myGoal]))
    {
      holonomic(0,0,30);
    }
  }
  else*/
  {
    if (pixy.updateBlocks() && pixy.sigSize[1]) //เห็นบอล
    {
      findGoalState = true;
      ballPosX = pixy.sigInfo[1][0].x;//รับตำแหน่งบอล
      ballPosY = pixy.sigInfo[1][0].y;
      for (int i = 0; i < 8; i++)
      {
        // get yaw angle
        getIMU();
      }
      if (discoveState) // ต้องไปหาบอล
      {
        goCatch(ballPosX, ballPosY, ballFli, fli_Kp1);
        oled.text(5,0,"catch to ball");
        oled.show();
  
        if ((abs(rot_error) <= rotErrorGap) && (abs(fli_error) <= flingErrorGap))
        {
          discoveState = false;
          oled.text(5,0,"got ball");
          oled.show();
          holonomic(0,0,0);
          unsigned long discoveTimer = millis();
        }
      }
      else
      {
        if (abs(ballFli - pixy.sigInfo[1][0].y) > flingErrorGap || abs(pixy.sigInfo[1][0].x - 162) > rotErrorGap) discoveState = true;
        else
        {
          if(getGoal(eneGoal))
          {
            ao();
            unsigned long loopTimer = millis();
            while (1)
            {
              holonomic(100, 90, 0);
              oled.text(5,0,"run to ball");
              oled.show();
             
              if (millis() - loopTimer >= 1000){
                break;
                }
            }/*
            shoot();
            holonomic(0,0,0);
            reload();*/
            discoveState = true;
          }
          {
            getIMU();
            if (pvYaw < 0)
            {
              while(!whiteLine() && pixy.updateBlocks() && (abs(ballFli - pixy.sigInfo[1][0].y) <= flingErrorGap) && (abs(pixy.sigInfo[1][0].x - 162) <= rotErrorGap))
              {
                ballPosY = pixy.sigInfo[1][0].y;
                ali_error = ballPosY - ballFli;
                ali_d = ali_error - ali_pError;
                ali_vec = ali_error * ali_Kp + ali_d * ali_Kd;
                ali_pError = ali_error;
                vecCurve = -ali_vec;
                radCurve = 27;
                holonomic(50, vecCurve, radCurve);
                oled.text(5,0,"vecCurve = %d", vecCurve );
                oled.show();
                if(getGoal(eneGoal))
                {
                  break;
                }
              }
          }
          else
          {
            while(!whiteLine() && pixy.updateBlocks() && (abs(ballFli - pixy.sigInfo[1][0].y) <= flingErrorGap) && (abs(pixy.sigInfo[1][0].x - 157) <= rotErrorGap))
            {
              ballPosY = pixy.sigInfo[1][0].y;
              ali_error = ballPosY - ballFli;
              ali_d = ali_error - ali_pError;
              ali_vec = ali_error * ali_Kp + ali_d * ali_Kd;
              ali_pError = ali_error;
              vecCurve = 180 + ali_vec;
              radCurve = -27;
              holonomic(50, vecCurve, radCurve);
              oled.text(5,0,"vecCurve = %d", vecCurve );
              oled.show();
              if(getGoal(eneGoal))
                {
                  break;
                }
            }
          } 
          }
        }
        }
    }
    else 
    {
      if(findGoalState)
      {
        if(pixy.updateBlocks() && pixy.sigSize[eneGoal])
        {
          goalNum = eneGoal;
          findGoalState = false;  
          goalDiscoveState = true;
        }
        else if(pixy.updateBlocks() && pixy.sigSize[myGoal])
        {
          goalNum = myGoal;
          findGoalState = false;  
          goalDiscoveState = true;
        }
        else
        {
          holonomic(0,0,30);
         oled.text(5,0,"find goal");
         oled.show();
        }
      }
      else if(goalDiscoveState && pixy.updateBlocks() && pixy.sigSize[goalNum])
      {
        goalPosX = pixy.sigInfo[goalNum][0].x;//รับตำแหน่งโกล
        goalPosY = pixy.sigInfo[goalNum][0].y;
        goalWidth = pixy.sigInfo[goalNum][0].width;
  
        goCatch(goalPosX, goalPosY, goalFli, fli_Kp2);
        oled.text(5,0,"catch to goal");
        oled.show();
        
        if ((abs(rot_error) <= rotErrorGap) && (abs(fli_error) <= flingErrorGap))
        {
          oled.text(5,0,"got goal");
          oled.show();
          if(goalNum == 3) goalNum = 2;
          else goalNum = 3; 
          goalDiscoveState = false;
          holonomic(0,0,0);
        } 
        
      }
      else
      {
       holonomic(0,0,30);
       oled.text(5,0,"find goal");
       oled.show();
       goalDiscoveState = true;
      }
    }
  }
}
