// File:          gebotController.cpp
// Date:
// Description:
// Author:songyifan
// Modifications:

#include <webots/Robot.hpp>
#include <gebotMotioncontrol.h>
#include <ctime>
#include <math.h>
#include <webots/Keyboard.hpp>
#define PI 3.1415926
using namespace webots;
CreepMotionControl mc;
int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Keyboard kb;
  kb.enable(1000);
  // int key;
  int timeStep = (int)robot->getBasicTimeStep(); 
  mc.initiation(robot);  
  mc.inverseKinematics();
  mc.setInitPos();  // set initposition
  // for(int i = 0; i<4; i++){
//       for(int j = 0; j<3; j++){
//             cout<<"ibss="<<mc.ftsPos(i,j)<<endl;
//             cout<<"ibss="<<mc.motorPos(i,j)<<endl;
//       }
//   }
    Emitter *emitter;
    emitter = robot->getEmitter("emitter"); 
  //emitter->enable(TIME_STEP);
    Vector<float, 4> tCV;
    tCV<< 0.0, 3.0, 0.0, 0.0;
    mc.setCoMVel(tCV);
    int answer;
  while (robot->step(timeStep) != -1)
  {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    // cout<<"presentTime="<<mc.presentTime<<endl;
    // cout<<"times="<<mc.times<<endl;
    // cout<<"comPos(0)="<<mc.comPos(0)<<endl;
     mc.nextStep();
    mc.inverseKinematics();
    mc.setJointPosition();
   // mc.turnback();
    // mc.sensorUpdate();
    // mc.forwardKinematics(); 

    // for(int i=0; i<4; i++){
    //   for(int j=0; j<3; j++)
    //   {
    //       if(mc.servoPos(i,j)>mc.jointPosmm(i*3+j,0))
    //       mc.jointPosmm(i*3+j,0)=mc.servoPos(i,j);
    //       if(mc.servoPos(i,j)<mc.jointPosmm(i*3+j,1))
    //       mc.jointPosmm(i*3+j,1)=mc.servoPos(i,j);

    
    //   }
    // }
    // for(int i=0; i<4; i++){
    //   for(int j=0; j<3; j++){
    //     cout<<"ftsPos"<<i*3+j<<"="<<mc.ftsPos(i,j)<<endl;
    //   }
    // }
    // for(int i=0; i<12; i++){
    //   for(int j=0; j<2; j++){
    //     cout<<"jointpos("<<i<<","<<j<<")="<<mc.jointPosmm(i,j)<<endl;
    //   }
    // }
    // if(mc.times>27)
    // for(int i=0; i<12; i++){
    //     float range[12];
    //     abab[i]=mc.jointPosmm(i,1)-mc.jointPosmm(i,0);
    //     cout<<"abab"<<i<<"="<<range[i]<<endl;
    // }

    // for(int i=0; i<4; i++){
    //   for(int j=0; j<3; j++){
    //     cout<<"shoulderPos("<<i<<","<<j<<")="<<mc.shoulderPos(i,j)<<endl;
    //   }
    // }
    // for(int i=0; i<4; i++){
    //   for(int j=0; j<3; j++){
    //     cout<<"footPos("<<i<<","<<j<<")="<<mc.footPos(i,j)<<endl;
    //   }
    // }

    // float temp_send[4][3];
    // for(int i=0; i<4; i++)
    // {
    //   for(int j=0; j<3; j++)
    //   {
    //     temp_send[i][j]=mc.Ts[i]->getValues()[j];
    //     cout << "Ts["<<i<<"]["<<j<<"]="<<temp_send[i][j]/7.4<<endl;
    //     // cout << temp_send[i][j]<<endl;
    //   }
    // }  // *getvalue &get location
    // cout <<temp_send[0][0]/7.4<<endl;
   // cout << "Ts1 = "<<temp_send[0][0]<<endl;//<<" Ts2 = "<< temp_send[1]<<" Ts3 = "<< temp_send[2]<<" Ts4 = "<<temp_send[3]<<endl;
    float gps[3];
    float imu[3];
    answer++;
     float abab=answer%4;
     if(abab==0){
    //    cout<<"times="<<mc.times<<endl;
    //    cout<<"presentTime="<<mc.presentTime<<endl;
        for(int i=0; i<3; i++)
        {
          gps[i]=mc.gps->getValues()[i];
          imu[i]=180/PI*mc.imu->getRollPitchYaw()[i];
        // cout << "gps["<<0<<"]="<<gps[0]<<endl;
        cout << "imu["<<i<<"]="<<imu[i]<<endl;
        }
     }
    //     for(int i=0; i<4; i++){
    //       for(int j=0; j<3; j++){
    //           cout<<"ftsPos("<<i<<","<<j<<")="<<mc.ftsPos(i,j)<<endl;
    //         }
    //      }
    //     cout <<"imu2="<<imu[2]<<endl;
    //  }
    //     // cout<<gps[2]<<endl;
    

    float timeSend[2];
    timeSend[0] = mc.presentTime;
    emitter->send(timeSend, sizeof(timeSend));
    // Enter here functions to send actuator commands, like:
    // motor->setPosition(10.0);
  }

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
