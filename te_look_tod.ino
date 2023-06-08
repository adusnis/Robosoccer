#include <POP32.h>
#include <POP32_Pixy2.h>
POP32_Pixy2 pixy;
#define degToRad 0.0175f
#define sin30 sin(30.f * degToRad)
#define cos30 cos(30.f * degToRad)
#define limPin PA0
#define reloadSpd 60
int timer = 0;
float ali_error, ali_pError, ali_d, ali_vec, vecCurve, radCurve;
int shootState = 1;

int Mode;
float thetaRad, vx, vy, spd1, spd2, spd3;
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
float pvYaw, lastYaw;
uint8_t rxCnt = 0, rxBuf[8];
bool getIMU() {
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
  }
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
  }
  motor(4, 0);
}
void setup() {
  oled.text(1,0,"READY");
  oled.show();
  while(1)
  {
    if(SW_OK())
    {
      Mode = 1;
      break;
    }
    if(SW_A())
    {
      Mode = 2;
      break;
    }
    if(SW_B())
    {
      Mode = 3;
      break;
    }
  }
}

void loop()
{
  oled.text(1,0,"DONE");
  oled.show();
  if (shootState)
  {
    if(Mode == 1)
    {
      holonomic(30, 200, -30);
      delay(130);
      ao();
      shoot();
      reload();
      shootState = 0;
    }
    else if(Mode == 2)
    {
      delay(130);
      shoot();
      reload();
      shootState = 0;
    }
    else if(Mode == 3)
    {
      holonomic(30, -20, 30);
      delay(130);
      ao();
      shoot();
      reload();
      shootState = 0;
    }
  }
}
