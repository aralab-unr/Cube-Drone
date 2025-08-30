#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <sbus.h> 
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

using namespace BLA;
#define SBUS_BAUD_RATE 100000
bfs::SbusRx sbus_rx(&Serial5);

#define NX          ACADO_NX    
#define NXA         ACADO_NXA   
#define NU          ACADO_NU    
#define N           ACADO_N    
#define NOD         ACADO_NOD   
#define NY          ACADO_NY  
#define NYN         ACADO_NYN   
#define NUM_STEPS   1        
#define VERBOSE     1          

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

float m = 1.85, g = 9.81;
float ch0 = 1000, ch1 = 1001, ch2 = 180, ch3 = 988, ch4 = 1090;
float pwm_value1 = 409, pwm_value2 = 409, pwm_value3 = 409, pwm_value4 = 409, pwm_value5 = 409, pwm_value6 = 409;

float phir = 0, thetar = 0, psir = 0;
float phird = 0, thetard = 0, psird = 0;

float cphi_ctrl = 2.5, ctheta_ctrl = 2.5, cpsi_ctrl = 3.5;
float Kx = 2.75, Ky = 2.75, Kz = 2.25;
float ixx = 0.0785, iyy = 0.0785, izz = 0.105;
float bphi = 1 / ixx;
float btheta = 1 / iyy;
float bpsi = 1 / izz;
float Kphi = 0.05, Ktheta = 0.05, Kpsi = 0.15;

float cx = 0.015, cy = 0.015, cz = 0.5;
float lam1 = 0.005, lam2 = 0.005, Ka = 0.15, eta = 0.005;
float Kdx = 0.0000267, Kdy = 0.0000267, Kdz = 0.0000625;

float xr = 0, yr = 0, zr = 0;

float vphi, vtheta, vpsi;
float vphicalibration, vthetacalibration, vpsicalibration;

int RateCalibrationNumber;
float AccX, AccY, AccZ;

float phi, theta, psi;
uint32_t LoopTimer;
float phikalman = 0, phiunkalman = 0.07;
float thetakalman = 0, thetaunkalman = 0.07;
float dt = 0.01;

float x = 0, y = 0, z = 0;
float xd = 0, yd = 0, zd = 0;
float ax = 0, ay = 0, az = 0;

float xkalman = 0, ykalman = 0, zkalman = 0;
float vxkalman = 0, vykalman = 0, vzkalman = 0;

BLA::Matrix<6,6> A; 
BLA::Matrix<6,3> B; 
BLA::Matrix<6,6> Q; 
BLA::Matrix<6,6> C; 
BLA::Matrix<6,6> R; 
BLA::Matrix<6,6> P; 
BLA::Matrix<6,6> K; 
BLA::Matrix<6,6> L; 

BLA::Matrix<6,1> esti;
BLA::Matrix<3,1> U;
BLA::Matrix<6,6> I;
BLA::Matrix<6,1> M;

float dtOptical = 0.02;

#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MAX_PAYLOAD_LEN 64
#define MICOLINK_MAX_LEN (MICOLINK_MAX_PAYLOAD_LEN + 7)

enum {
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,
};

typedef struct {
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    // internal parsing state
    uint8_t status = 0;
    uint8_t payload_cnt = 0;
} MICOLINK_MSG_t;

#pragma pack(push, 1)
typedef struct {
    uint32_t time_ms;
    uint32_t distance;
    uint8_t strength;
    uint8_t precision;
    uint8_t dis_status;
    uint8_t reserved1;
    int16_t flow_vel_x;
    int16_t flow_vel_y;
    uint8_t flow_quality;
    uint8_t flow_status;
    uint16_t reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack(pop)

void pid_controller(float &Uout, float &integral, float &pre_error, float error, float kp, float ki, float kd, float dt) {
  integral += error * dt;
  integral = constrain(integral, -1.0, 1.0);
  float derivative =  (error - pre_error) / dt;
  Uout = kp * error + ki * integral + kd * derivative;
  Uout = constrain(Uout, -100.0, 100.0);
  pre_error = error;
}

void kalman_position(void){
  U = {ax,
       ay,
       az};
  esti = A * esti + B * U;
  P = A * P * ~A + Q;
  L = C * P * ~C + R;
  K = P * ~C * Invert(L);
  M={x,
     y,
     z,
     xd,
     yd,
     zd};
  esti = esti + K * (M - C * esti);
  P = (I - K * C) * P;
  xkalman = esti(0,0);
  ykalman = esti(1,0);
  zkalman = esti(2,0);
  vxkalman = esti(3,0);
  vykalman = esti(4,0);
  vzkalman = esti(5,0);
}

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    vphicalibration += vphi;
    vthetacalibration += vtheta;
    vpsicalibration += vpsi;
    delay(1);
  }
  
  vphicalibration /= 2000;
  vthetacalibration /= 2000;
  vpsicalibration /= 2000;

  A = {1, 0, 0, 0.01, 0, 0,
       0, 1, 0, 0, 0.01, 0,
       0, 0, 1, 0, 0, 0.01,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1};

  B = {0.00005, 0, 0,
       0, 0.00005, 0,
       0, 0, 0.00005,
       0.01, 0, 0,
       0, 0.01, 0,
       0, 0, 0.01};

  C = {1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1};
  
  Q = {0.01, 0, 0, 0, 0, 0,
       0, 0.01, 0, 0, 0, 0,
       0, 0, 0.04, 0, 0, 0,
       0, 0, 0, 0.01, 0, 0,
       0, 0, 0, 0, 0.01, 0,
       0, 0, 0, 0, 0, 0.25};
  
  R = {0.000004, 0, 0, 0, 0, 0,
       0, 0.000004, 0, 0, 0, 0,
       0, 0, 0.009, 0, 0, 0,
       0, 0, 0, 0.000004, 0, 0,
       0, 0, 0, 0, 0.000004, 0,
       0, 0, 0, 0, 0, 0.01};

  I = {1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1};

  esti = {0,
        0,
        0,
        0,
        0,
        0};

  Serial5.begin(SBUS_BAUD_RATE, SERIAL_8E2);
  sbus_rx.Begin();
  Serial3.begin(115200);
  acado_initializeSolver();

  analogWriteFrequency(3, 100);
  analogWriteFrequency(4, 100);
  analogWriteFrequency(5, 100);
  analogWriteFrequency(6, 100);
  analogWriteFrequency(9, 100);
  analogWriteFrequency(2, 100);
  analogWriteFrequency(28, 100);
  analogWriteFrequency(29, 100);
  analogWriteFrequency(36, 100);
  analogWriteFrequency(37, 100);
  analogWriteResolution(12);

  analogWrite(3, 409); 
  analogWrite(4, 409); 
  analogWrite(5, 409); 
  analogWrite(6, 409); 
  analogWrite(9, 409); 
  analogWrite(2, 409); 
  analogWrite(28, 409); 
  analogWrite(29, 409); 
  analogWrite(36, 409); 
  analogWrite(37, 409); 
  delay(3000);

  LoopTimer = micros();
}

void loop() {

  read_receiver();
  gyro_signals();
  getoptical();
  vphi -= vphicalibration;
  vtheta -= vthetacalibration;
  vpsi -= vpsicalibration;
  
  kalman_angle(phikalman, phiunkalman, vphi, phi);
  kalman_angle(thetakalman, thetaunkalman, vtheta, theta);

  ax = -AccX * 9.81;
  ay = -AccY * 9.81;
  az = (AccZ * cos(phikalman) * cos(thetakalman) - 1) * 9.81;

  R = {0.0001 * zkalman * zkalman, 0, 0, 0, 0, 0,
       0, 0.0001 * zkalman * zkalman, 0, 0, 0, 0,
       0, 0, 0.0004 * zkalman * zkalman, 0, 0, 0,
       0, 0, 0, 0.0001 * zkalman * zkalman, 0, 0,
       0, 0, 0, 0, 0.0001 * zkalman * zkalman, 0,
       0, 0, 0, 0, 0, 0.0004 * zkalman * zkalman};

  kalman_position();

  zr = map(ch2, 180, 1811, 0.0, 2.0);
  xr = xr + (1000 - ch1) / 1200 * 0.01;
  yr = yr + (1001 - ch0) / 1200 * 0.01;
  psir = psir + 0.00005 * (ch3 - 988);

  float thetar = 0.125 * exp(-0.25 * zkalman) * (xr - xkalman)  - 0.2 * exp(-0.5 * zkalman) * vxkalman;
  float phir = 0.125 * exp(-0.25 * zkalman) * (ykalman - yr) + 0.2 * exp(-0.5 * zkalman) * vykalman;
  phir = constrain(phir, -0.3, 0.3);
  thetar = constrain(thetar, -0.3, 0.3);

  phird = 6.25 * (phir - phikalman) - 0.125 * vphi;
  thetard = 6.25 * (thetar - thetakalman) - 0.125 * vtheta;

  psird = 0.001 * (ch3 - 988);

  phird = constrain(phird, -0.75, 0.75);
  thetard = constrain(thetard, -0.75, 0.75);
  psird = constrain(psird, -1.0, 1.0);

  float fphi = vphi * vtheta * (iyy - izz) / ixx;
  float ftheta = vpsi * vphi * (izz - ixx) / iyy;
  float fpsi = vphi * vtheta * (ixx - iyy) / izz;

  float scphi = cphi_ctrl * (phikalman - phir) + (vphi - phird);
  float sctheta = ctheta_ctrl * (thetakalman - thetar) + (vtheta - thetard);
  float scpsi = cpsi_ctrl * (psi - psir) + (vpsi - psird);

  float satphi = constrain(scphi, -0.1, 0.1);
  float sattheta = constrain(sctheta, -0.1, 0.1);
  float satpsi = constrain(scpsi, -0.1, 0.1);

  float U2s = (-Ky * cphi_ctrl * phikalman - (cphi_ctrl + Ky) * vphi + Ky * cphi_ctrl * phir + (cphi_ctrl + Ky) * phird - fphi - Kphi * satphi) / bphi;
  float U3s = (-Kx * ctheta_ctrl * thetakalman - (ctheta_ctrl + Kx) * vtheta + Kx * ctheta_ctrl * thetar + (ctheta_ctrl + Kx) * thetard - ftheta - Ktheta * sattheta) / btheta;
  float U4s = (-Kz * cpsi_ctrl * psi - (cpsi_ctrl + Kz) * vpsi + Kz * cpsi_ctrl * psir + (cpsi_ctrl + Kz) * psird - fpsi - Kpsi * satpsi) / bpsi;

  float U2smc = constrain(U2s, -1.5, 1.5);
  float U3smc = constrain(U3s, -1.5, 1.5);
  float U4smc = constrain(U4s, -1.25, 1.25);

  float cphi = cos(phikalman), sphi = sin(phikalman);
  float ctheta = cos(thetakalman), stheta = sin(thetakalman);
  float cpsi = cos(psi), spsi = sin(psi);

  float fx = -Kdx * vxkalman / m;
  float fy = -Kdy * vykalman /m;
  float fz = ( -Kdz * vzkalman - m * g) / m;
  float bx = 1 / m * (spsi * sphi + cpsi * stheta * cphi);
  float by = 1 / m * (spsi * stheta * cphi - cpsi * sphi);
  float bz = 1 / m * (ctheta * cphi);

  float xrd = 0.05 * (xr - xkalman);
  float yrd = 0.05 * (yr - ykalman);
  float zrd = 0.5 * (zr - zkalman);

  zrd = constrain(zrd, -0.3, 0.3); 

  float sx = cx * (xr - xkalman) + (xrd - vxkalman);
  float sy = cy * (yr - ykalman) + (yrd - vykalman);
  float sz = cz * (zr - zkalman) + 0.5 * (zrd - vzkalman);

  float ueqx = (cx * (xrd - xkalman) - fx) / bx;
  float ueqy = (cy * (yrd - ykalman) - fy) / by;
  float ueqz = (cz * (zrd - zkalman) - fz) / bz;

  float s3 = lam2 * lam1 * sx + lam2 * sy + sz;
  float sats3 = constrain(s3, -0.1, 0.1);

  float usw = -(lam2 * lam1 * bx * (ueqy + ueqz) + lam2 * by * (ueqx + ueqz) + bz * (ueqx + ueqy) - Ka * s3 - eta * sats3) / (lam2 * lam1 * bx + lam2 * by + bz);
  float Uz = ueqx + ueqy + ueqz + usw;
  float U1smc = constrain(Uz, 15.5, 21.5);

  unsigned int i;

  memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  memset(&acadoVariables, 0, sizeof(acadoVariables));

  for (i = 0; i < N + 1; ++i) {
      acadoVariables.x[i * NX + 0] = xkalman;
      acadoVariables.x[i * NX + 1] = ykalman;
      acadoVariables.x[i * NX + 2] = zkalman;
      acadoVariables.x[i * NX + 3] = phikalman;
      acadoVariables.x[i * NX + 4] = thetakalman;
      acadoVariables.x[i * NX + 5] = psi;
      acadoVariables.x[i * NX + 6] = vxkalman;
      acadoVariables.x[i * NX + 7] = vykalman;
      acadoVariables.x[i * NX + 8] = vzkalman;
      acadoVariables.x[i * NX + 9] = vphi;
      acadoVariables.x[i * NX + 10] = vtheta;
      acadoVariables.x[i * NX + 11] = vpsi;
  }

  for (i = 0; i < N; ++i) {
      acadoVariables.y[i * NY + 0] = zr;
      acadoVariables.y[i * NY + 1] = phir;
      acadoVariables.y[i * NY + 2] = thetar; 
      acadoVariables.y[i * NY + 3] = psir;
      acadoVariables.y[i * NY + 4] = U1smc;
      acadoVariables.y[i * NY + 5] = U2smc;
      acadoVariables.y[i * NY + 6] = U3smc;
      acadoVariables.y[i * NY + 7] = U4smc;
  }

  acadoVariables.yN[0] = zr;
  acadoVariables.yN[1] = phir; 
  acadoVariables.yN[2] = thetar; 
  acadoVariables.yN[3] = psir;

  for (i = 0; i < NX; ++i) {
      acadoVariables.x0[i] = acadoVariables.x[i];
  }

  acado_preparationStep();
  acado_feedbackStep();
  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  real_t* u = acado_getVariablesU();
  
  float U1r = map(u[0] - m*g, -3, 3, -125, 125);
  float U1manual = map(ch2, 180, 1811, 1000, 1800);
  float U2r = 200 * u[1];
  float U3r = 200 * u[2];
  float U4r = 200 * u[3];

  while (micros() - LoopTimer < 10000);
  dt = (micros() - LoopTimer) / 1000000.0;
  LoopTimer = micros();
  psi = psi + dt * vpsi;

  if (ch4 < 700) {
    pwm_value1 = constrain(0.409 * (1550 + U1r - U2r - U3r - U4r), 409, 800);
    pwm_value2 = constrain(0.409 * (1550 + U1r + U2r - U3r + U4r), 409, 800);
    pwm_value3 = constrain(0.409 * (1550 + U1r - U2r + U3r + U4r), 409, 800);
    pwm_value4 = constrain(0.409 * (1550 + U1r + U2r + U3r - U4r), 409, 800);
  }
  else if (ch4 >= 700 && ch4 < 1000) {
    pwm_value1 = constrain(0.409 * (U1manual - U2r - U3r - U4r), 409, 800);
    pwm_value2 = constrain(0.409 * (U1manual + U2r - U3r + U4r), 409, 800);
    pwm_value3 = constrain(0.409 * (U1manual - U2r + U3r + U4r), 409, 800);
    pwm_value4 = constrain(0.409 * (U1manual + U2r + U3r - U4r), 409, 800);
  }
  else {
    pwm_value1 = 409;
    pwm_value2 = 409;
    pwm_value3 = 409;
    pwm_value4 = 409; 
  }

  analogWrite(6, (int)pwm_value1); //motor 1
  analogWrite(4, (int)pwm_value2); //motor 2
  analogWrite(5, (int)pwm_value3); //motor 3
  analogWrite(3, (int)pwm_value4); //motor 4

  analogWrite(28, (int)pwm_value1); //motor 5
  analogWrite(2, (int)pwm_value2); //motor 6
  analogWrite(29, (int)pwm_value3); //motor 7
  analogWrite(9, (int)pwm_value4); //motor 8

  analogWrite(36, (int)pwm_value5); //motor 9
  analogWrite(37, (int)pwm_value6); //motor 10

  Serial.print(U1manual);Serial.print(",");
  Serial.print((int)pwm_value2);Serial.print(",");
  Serial.print((int)pwm_value3);Serial.print(",");
  Serial.print((int)pwm_value4);Serial.println(",");
}

void kalman_angle(float &angleesti, float &angleun, float anglein, float vangle) {
  angleesti = angleesti + 0.01 * anglein;
  angleun = angleun + 0.01 * 0.01 * 0.035 * 0.035;
  float kgain = angleun / (angleun + 0.00275); 
  angleesti = angleesti + kgain * (vangle - angleesti);
  angleun = (1 - kgain) * angleun;
}

void read_receiver() {
    if (sbus_rx.Read()) {
        bfs::SbusData data = sbus_rx.data();
        ch0 = data.ch[0];
        ch1 = data.ch[1];
        ch2 = data.ch[2];
        ch3 = data.ch[3];
        ch4 = data.ch[4];
    }
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  vphi = (float)GyroX / 65.5 * PI / 180.0;
  vtheta = (float)GyroY / 65.5 * PI / 180.0;
  vpsi = (float)GyroZ / 65.5 * PI / 180.0;
  
  AccX=(float)AccXLSB/4096 - 0.05;
  AccY=(float)AccYLSB/4096 + 0.01;
  AccZ=(float)AccZLSB/4096 - 0.1175;
  
  phi = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ));
  theta = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ));
}

void getoptical() {
  while (Serial3.available()) {
    uint8_t sensor_data = Serial3.read();
    micolink_decode(sensor_data);
  }
}

void micolink_decode(uint8_t data) {
  static MICOLINK_MSG_t msg;

  if (!micolink_parse_char(&msg, data)) return;

  if (msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR) {
    MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
    memcpy(&payload, msg.payload, msg.len);

    if (payload.distance < 8000 && fabs(payload.flow_vel_x) < 1500 && fabs(payload.flow_vel_y) < 1500) {
      if (dtOptical > 0.0) {
        zd = (payload.distance / 1000.0 * cos(phikalman) * cos(thetakalman) - z) / dtOptical;
        z = payload.distance / 1000.0 * cos(phikalman) * cos(thetakalman);

        xd = payload.flow_vel_x * z * 0.01;
        yd = -payload.flow_vel_y * z * 0.01;

        x = x + xd * dtOptical;
        y = y + yd * dtOptical;
      }
    }
  }
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data) {
  switch (msg->status) {
    case 0:
      if (data == MICOLINK_MSG_HEAD) {
        msg->head = data;
        msg->status = 1;
      }
      break;
    case 1: msg->dev_id = data; msg->status = 2; break;
    case 2: msg->sys_id = data; msg->status = 3; break;
    case 3: msg->msg_id = data; msg->status = 4; break;
    case 4: msg->seq = data; msg->status = 5; break;
    case 5:
      msg->len = data;
      if (msg->len == 0) {
        msg->status = 7;
      } else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN) {
        msg->status = 0;
      } else {
        msg->payload_cnt = 0;
        msg->status = 6;
      }
      break;
    case 6:
      msg->payload[msg->payload_cnt++] = data;
      if (msg->payload_cnt == msg->len) {
        msg->status = 7;
      }
      break;
    case 7:
      msg->checksum = data;
      msg->status = 0;
      return true;
    default:
      msg->status = 0;
      msg->payload_cnt = 0;
      break;
  }
  return false;
}


