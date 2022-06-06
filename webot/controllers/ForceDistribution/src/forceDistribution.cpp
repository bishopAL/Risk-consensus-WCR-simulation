#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <stdio.h>
#include <stdlib.h>
#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/Emitter.hpp>
#include <ctime>
#include <webots/Keyboard.hpp>
#define PI 3.1415926
#define TIME_STEP 10
using namespace webots;
using namespace std;
using namespace Eigen;

PositionSensor *Ps[12];
Motor *Tor[12];
TouchSensor *Ts[4];
InertialUnit *imu;
GPS *gps;
// LF, RF, LH, RH
char positionName[12][22] = {"LF0 PositionSensor", "LF1 PositionSensor", "LF2 PositionSensor", "RF0 PositionSensor", "RF1 PositionSensor", "RF2 PositionSensor", "LH0 PositionSensor", "LH1 PositionSensor", "LH2 PositionSensor", "RH0 PositionSensor", "RH1 PositionSensor", "RH2 PositionSensor"};
char motorName[12][22] = {"LF0 RotationalMotor", "LF1 RotationalMotor", "LF2 RotationalMotor", "RF0 RotationalMotor", "RF1 RotationalMotor", "RF2 RotationalMotor", "LH0 RotationalMotor", "LH1 RotationalMotor", "LH2 RotationalMotor", "RH0 RotationalMotor", "RH1 RotationalMotor", "RH2 RotationalMotor"};
char touchsensorName[4][16] = {"LF_touch_sensor", "RF_touch_sensor", "LH_touch_sensor", "RH_touch_sensor"};
int controlPeriod=0;
float footForce[4][3];
float jointPos[12];
float jointVel[12];
float pstFootPos[4][3];
float pstFootVel[4][3];
float targetFootPos[4][3];
float L1 = 50;
float L2 = 47;
float L3 = 40;  
float xc_dotdot[4];
float xc_dot[4];
float M = 1;
float K = 10;
float B = 1;
float refForce[4] = {0};
Matrix<float, 4 ,3> cmdFootPos;  // LF RF LH RH X, Y, Z foot_to_shoulder
Matrix<float, 4 ,3> cmdJointPos;
int main(int argc, char **argv) {
    Robot *robot = new Robot();
    Keyboard kb;
    // Emitter *emitter;
    // emitter = robot->getEmitter("emitter"); 
    // emitter->enable(TIME_STEP);
    for (int i=0; i<12; i++)
    {
      Ps[i] = robot->getPositionSensor(positionName[i]);
      Ps[i]->enable(TIME_STEP); 
      Tor[i] = robot->getMotor(motorName[i]);
    }
    for (int i=0; i<4; i++)
    {
      Ts[i] = robot->getTouchSensor(touchsensorName[i]);
      Ts[i]->enable(TIME_STEP);   
    }
    while (robot->step(TIME_STEP) != -1)
    {
        controlPeriod++;
        cout<<"controlPeriod: "<<controlPeriod<<endl;
        // Get Force
        for(int i=0; i<4; i++)
        {
          cout<<i<<" Foot Force: ";
          for(int j=0; j<3; j++)
          {
            footForce[i][j]=Ts[i]->getValues()[j]/7.3;
            cout<<footForce[i][j]<<", ";
          }
          cout<<"; ";
        }
        cout<<"Force Sum: "<<footForce[0][0] + footForce[1][0] + footForce[2][0] + footForce[3][0]<<"."<<endl;

        // Get Pos & Vel
        for(int i=0; i<12; i++)
        {
          float tempPos[12];
          tempPos[i] = Ps[i]->getValue();
          jointVel[i] = (tempPos[i] - jointPos[i]) / (float(TIME_STEP) / 1000);
          jointPos[i] = tempPos[i];
          cout<<jointPos[i]<<", "<<jointVel[i]<<", ";
        }   
        cout<<". "<<endl;

        // FK


        // Admittance control
        for(uint i=0;i<3;i++)
        {
          xc_dotdot[i] = 0 + M/-1*(footForce[i][0] - refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos[i][0]));
          xc_dot[i] = pstFootPos[i][0] + xc_dotdot[i] * float(TIME_STEP) / 1000;
          cmdFootPos(i,0) = pstFootPos[i][0] + xc_dot[i] * float(TIME_STEP) / 1000;
        }
        

        // IK
        for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
        {
          float factor_y, factor_x, factor_xc, factor_yc, factor_zc;  // factor for x/y; factor for whole formula
          if(legNum==0)
          {
              factor_xc=-1;
              factor_yc=1;
              factor_zc=1;
              factor_x=1;
              factor_y=1;
          }
          if(legNum==1)
          {
              factor_xc=1;
              factor_yc=-1;
              factor_zc=-1;
              factor_x=1;
              factor_y=-1;
          }
          if(legNum==2)
          {
              factor_xc=-1;
              factor_yc=-1;
              factor_zc=-1;
              factor_x=-1;
              factor_y=1;
          }
          if(legNum==3)
          {
              factor_xc=1;
              factor_yc=1;
              factor_zc=1;
              factor_x=-1;
              factor_y=-1;
          }
          cmdJointPos(legNum,1) = -factor_xc * (asin(L3 / sqrt( cmdFootPos(legNum,2)*cmdFootPos(legNum,2) + cmdFootPos(legNum,1)*cmdFootPos(legNum,1) )) + atan2(cmdFootPos(legNum,2),factor_y * cmdFootPos(legNum,1)) );     
          cmdJointPos(legNum,0) = -factor_yc * (asin((cmdFootPos(legNum,1) * cmdFootPos(legNum,1) + cmdFootPos(legNum,0) * cmdFootPos(legNum,0) + cmdFootPos(legNum,2) * cmdFootPos(legNum,2) + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (cmdFootPos(legNum,1) * cmdFootPos(legNum,1) +  cmdFootPos(legNum,0) * cmdFootPos(legNum,0) + cmdFootPos(legNum,2) * cmdFootPos(legNum,2) - L3 * L3)))
                  - atan2(sqrt(cmdFootPos(legNum,1) * cmdFootPos(legNum,1) + cmdFootPos(legNum,2) * cmdFootPos(legNum,2) - L3 * L3) , factor_x * cmdFootPos(legNum,0)));
          cmdJointPos(legNum,2) = -factor_zc * asin((L1 * L1 + L2 * L2 + L3 * L3 - cmdFootPos(legNum,1) * cmdFootPos(legNum,1) - cmdFootPos(legNum,0) * cmdFootPos(legNum,0) - cmdFootPos(legNum,2) * cmdFootPos(legNum,2)) / (2 * L1 * L2));
        }
        cout<<"----------------------"<<endl;
        

    }
}

