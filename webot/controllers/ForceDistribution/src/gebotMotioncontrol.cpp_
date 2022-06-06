#include <gebotMotioncontrol.h>
#include <math.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#define PI 3.1415926
#define TIME_STEP 10
using namespace std;
using namespace Eigen;


float timePeriod=0.01;
float T=4;  // whole Time
float timeGait;  // time for one step
float presentTime=0; //present time
void CreepMotionControl::initiation(Robot *robot)
{
    
    times = 0;
    width = 60.5;
    length = 124.0;
    L1 = 50;
    L2 = 47;
    L3 = 40;  
    timeGait = T; 

    comPos << 0.0, 0.0, 0.0, 0.0;
    comPos_bymotor << 0.0, 0.0, 0.0, 0.0;
   // timeForStancePhase << 0, 2, 2, 4, 2, 4, 0, 2;
    timeForSwingPhase << 0.5*T, 0.95*T, 0*T, 0.45*T, 0*T, 0.45*T, 0.5*T, 0.95*T;
   // swingTime << 0.25*T, 0.25*T, 0.25*T, 0.25*T;
    shoulderPos << length/2, width/2, 0, 1, length/2, -width/2, 0, 1, -length/2, width/2, 0, 1, -length/2, -width/2, 0, 1;  // X-Y-Z-alpha: LF, RF, LH, RH
    footPos << L2+length/2, L1+width/2, -L3, L2+length/2, -L1-width/2, -L3, -L2-length/2, L1+width/2, -L3, -L2-length/2, -L1-width/2, -L3;  // X-Y-Z: LF, RF, LH, RH构型
    ftsPos << 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0;
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            ftsPos(i,j) = footPos(i,j) - shoulderPos(i,j);
        }
    }
    targetCoMVelocity << 0.0, 0.0, 0.0, 0.0;
    timeGait = T; 
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
   imu = robot->getInertialUnit("inertial unit");
   imu->enable(TIME_STEP);
   gps = robot->getGPS("gps");
   gps->enable(TIME_STEP);
}
void CreepMotionControl::setInitPos()
{
    int joints = 0;
    for(int i = 0; i<4; i++)
    {
        for(int j = 0; j<3; j++){
            jointPos[joints] = motorPos(i,j);
            joints++;}
    }
    for (int i = 0; i < 12; i++){
        Tor[i]->setPosition(jointPos[i]);}
}
void CreepMotionControl::setCoMVel(Vector<float, 4> cmdV)
{
    targetCoMVelocity = cmdV;
}
void CreepMotionControl::nextStep()
{
    if(times>=0&&times<2)   
    {
        for(int i=0; i<4; i++)
        {
            // comPos(i) = comPos(i) + comPos(i) - comPos_bymotor(i) + timePeriod * targetCoMVelocity(i); // 获取新质心坐标（在世界坐标系中）
            comPos(i) = comPos(i) + timePeriod * targetCoMVelocity(i);
        }
        for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
        {   // for swingphase 
            if(presentTime > timeForSwingPhase(legNum, 0)-timePeriod/2 && presentTime < timeForSwingPhase(legNum, 1)+timePeriod/2) 
            {  
                timeForSwing(legNum) = timeForSwingPhase(legNum,1) - timeForSwingPhase(legNum,0);  // time last for swingphase 
            // cout<<"time for swing"<<int(legNum)<<"="<<timeForSwing(legNum)<<endl;
                Vector<float, 3> swingphaseVelocity = 1.333 * (stancePhaseEndPos.row(legNum) - stancePhaseStartPos.row(legNum)) / timeForSwing(legNum);
            // cout<<"swingphaseVelocity="<<swingphaseVelocity(0)<<endl;
                float onestepH = 20;
                // float onestepL = abs(stancePhaseEndPos(legNum,0) - stancePhaseStartPos(legNum,0));  // this is minus!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!make it positive      
                float onestepW = abs(stancePhaseEndPos(legNum,1) - stancePhaseStartPos(legNum,1));   
             //   cout<<"onestepW="<<onestepW<<endl;
                if(times==0&&(legNum==1||legNum==2))  // trot tag
                {
                    if(presentTime-timeForSwingPhase(legNum, 0) > -timePeriod/2 && presentTime-timeForSwingPhase(legNum, 0)<timeForSwing(legNum)/2 + timePeriod)
                    {
                        ftsPos(legNum, 2)=ftsPos(legNum, 2) + timePeriod*20;
                        footPos(legNum, 0) = ftsPos(legNum, 0) + shoulderPos(legNum, 0) + comPos(0);
                        footPos(legNum, 1) = ftsPos(legNum, 1) + shoulderPos(legNum, 1) + comPos(1);
                    }
                    if(presentTime-timeForSwingPhase(legNum, 0) > timeForSwing(legNum)/2-timePeriod/2 && presentTime-timeForSwingPhase(legNum, 1)< timePeriod)
                    {
                        ftsPos(legNum, 2)=ftsPos(legNum, 2) - timePeriod*20;
                        footPos(legNum, 0) = ftsPos(legNum, 0) + shoulderPos(legNum, 0) + comPos(0);
                        footPos(legNum, 1) = ftsPos(legNum, 1) + shoulderPos(legNum, 1) + comPos(1);
                    }
                }
                else  // trot
                {                
                    if(presentTime-timeForSwingPhase(legNum, 0) > -timePeriod/2 && presentTime-timeForSwingPhase(legNum, 0) < timeForSwing(legNum)/2+timePeriod/2)
                    {
                        ftsPos(legNum, 0) = ftsPos(legNum, 0) - swingphaseVelocity(0) * timePeriod;
                        footPos(legNum, 0) = ftsPos(legNum, 0) + shoulderPos(legNum, 0) + comPos(0);
                        ftsPos(legNum, 1) = ftsPos(legNum, 1) - swingphaseVelocity(1) * timePeriod;
                        footPos(legNum, 1) = ftsPos(legNum, 1) + shoulderPos(legNum, 1) + comPos(1);
                        if(legNum==0||legNum==2)
                        {                    
                            // ftsPos(legNum, 2) = -9/4 * onestepH/(onestepL*onestepL)*(ftsPos(legNum, 0)-L2+onestepL/3)*(ftsPos(legNum, 0)-L2+onestepL/3)+onestepH -L3;
                            ftsPos(legNum, 2) = -9/4 * onestepH/(onestepW*onestepW)*(ftsPos(legNum, 1)-L1+onestepW/3)*(ftsPos(legNum, 1)-L1+onestepW/3)+onestepH -L3;
                        } // Z's direction movement
                        if(legNum==1||legNum==3)
                        {
                            // ftsPos(legNum, 2) = -9/4 * onestepH/(onestepL*onestepL)*(ftsPos(legNum, 0)+L2+onestepL/3)*(ftsPos(legNum, 0)+L2+onestepL/3)+onestepH -L3;
                            ftsPos(legNum, 2) = -9/4 * onestepH/(onestepW*onestepW)*(ftsPos(legNum, 1)+L1+onestepW/3)*(ftsPos(legNum, 1)+L1+onestepW/3)+onestepH -L3;
                        } // Z's direction movement
                        // if(times == 0 && legNum == 1)ftsPos(legNum,0)=47;
                        // if(timeForSwingPhase(2,0) == timeForSwingPhase(0,0) && times == 0)ftsPos(2,0)=-47;              
                    }
                    if(presentTime-timeForSwingPhase(legNum, 0) > timeForSwing(legNum)/2 && presentTime-timeForSwingPhase(legNum, 0) < timeForSwing(legNum)*0.75 + timePeriod/2)
                    {
                        ftsPos(legNum, 0) = ftsPos(legNum, 0) - swingphaseVelocity(0) * timePeriod;
                        footPos(legNum, 0) = ftsPos(legNum, 0) + shoulderPos(legNum, 0) + comPos(0);
                        ftsPos(legNum, 1) = ftsPos(legNum, 1) - swingphaseVelocity(1) * timePeriod;
                        footPos(legNum, 1) = ftsPos(legNum, 1) + shoulderPos(legNum, 1) + comPos(1);
                        if(legNum==0 || legNum==2){
                        // ftsPos(legNum, 2) = sqrt(onestepL*onestepL/9-(ftsPos(legNum, 0)-L2+onestepL/3)*(ftsPos(legNum, 0)-L2+onestepL/3)) + onestepH - onestepL/3 -L3;
                        ftsPos(legNum, 2) = -9/4 * onestepH/(onestepW*onestepW)*(ftsPos(legNum, 1)-L1+abs(onestepW/3))*(ftsPos(legNum, 1)-L1+abs(onestepW/3))+onestepH -L3;} // Y's direction movement
                        if(legNum==1 || legNum==3){
                        // ftsPos(legNum, 2) = sqrt(onestepL*onestepL/9-(ftsPos(legNum, 0)+L2+onestepL/3)*(ftsPos(legNum, 0)+L2+onestepL/3)) + onestepH - onestepL/3 -L3;
                        ftsPos(legNum, 2) = -9/4 * onestepH/(onestepW*onestepW)*(ftsPos(legNum, 1)+L1+abs(onestepW/3))*(ftsPos(legNum, 1)+L1+abs(onestepW/3))+onestepH -L3;} // Y's direction movement
                    }
                    if(presentTime-timeForSwingPhase(legNum, 0) > timeForSwing(legNum)*0.75 + timePeriod/2 && presentTime-timeForSwingPhase(legNum, 1) < timePeriod/2)
                    {
                        footPos(legNum, 1) = footPos(legNum, 1) + targetCoMVelocity(1) * timePeriod;  //still moving
                        // ftsPos(legNum, 2) = ftsPos(legNum, 2) - 4*(onestepH - onestepL/3)/timeForSwing(legNum) * timePeriod;
                        ftsPos(legNum, 2) = ftsPos(legNum, 2) - 4*(onestepH - onestepW/3)/timeForSwing(legNum) * timePeriod;
                    }

                }
            } 
            // for stancephase
            if(presentTime - timeForSwingPhase(legNum, 0) < timePeriod/2 || presentTime > timeForSwingPhase(legNum, 1)+timePeriod/2)
            {                   
                if(times == 0)
                {
                    if(presentTime < timePeriod/2) // 记录起始坐标
                    {
                        stancePhaseStartPos(legNum,0) = ftsPos(legNum,0);
                        stancePhaseStartPos(legNum,1) = ftsPos(legNum,1);
                        stancePhaseStartPos(legNum,2) = ftsPos(legNum,2);
                    }              
                }
                if(times !=0 )
                {      
                    if(abs(presentTime - timeForSwingPhase(legNum, 1)) < timePeriod/2) // 记录起始坐标
                    {
                        stancePhaseStartPos(legNum,0) = ftsPos(legNum,0);
                        stancePhaseStartPos(legNum,1) = ftsPos(legNum,1);
                        stancePhaseStartPos(legNum,2) = ftsPos(legNum,2);                   
                    }
                }
                if(abs(presentTime - timeForSwingPhase(legNum, 0)) < timePeriod/2)  // 记录终点坐标
                {
                    stancePhaseEndPos(legNum,0) = ftsPos(legNum,0);
                    stancePhaseEndPos(legNum,1) = ftsPos(legNum,1);
                    stancePhaseEndPos(legNum,2) = ftsPos(legNum,2);
                } 
                Matrix<float, 4, 4> trans;
                trans<<cos(comPos(3)), -sin(comPos(3)), 0, comPos(0),
                    sin(comPos(3)), cos(comPos(3)), 0, comPos(1),
                    0, 0, 1, 0,
                    0, 0, 0, 1;
                Matrix<float, 4, 1> oneShoulderPos_4x1;
                oneShoulderPos_4x1 << shoulderPos.row(legNum)(0), shoulderPos.row(legNum)(1), 0, 1;
                oneShoulderPos_4x1 = trans * oneShoulderPos_4x1;  // 获取新关节坐标（在世界坐标系中）
                ftsPos(legNum,0) = footPos(legNum,0) - oneShoulderPos_4x1(0);  // X
                ftsPos(legNum,1) = footPos(legNum,1) - oneShoulderPos_4x1(1);  // Y
                ftsPos(legNum,2) = footPos(legNum,2);  // Z
                // cout<<"shoulderPos.row(legNum)(0)="<<shoulderPos.row(legNum)(0)<<endl;
            // if(legNum==1)cout<<"oneShoulderPos_4x1(0)="<<oneShoulderPos_4x1(0)<<endl;
                // cout<<"oneShoulderPos_4x1(1)="<<oneShoulderPos_4x1(1)<<endl;
                // cout<<"oneShoulderPos_4x1(2)="<<oneShoulderPos_4x1(2)<<endl;
            } 
        } 
    } 
    if(times==2) //turn
    {
        roll = 0.001;
        for(uint8_t legNum=0; legNum<4; legNum++){
            if(legNum==0||legNum==2){       
            ftsPos(legNum,2) = ftsPos(legNum,2)-width/2*sin(roll);
            ftsPos(legNum,1) = ftsPos(legNum,1)+width/2*(1-cos(roll));}
            if(legNum==1||legNum==3){
            ftsPos(legNum,2) = ftsPos(legNum,2)+width/2*sin(roll);
            ftsPos(legNum,1) = ftsPos(legNum,1)-width/2*(1-cos(roll));
            }
        }   
        // cout<<"sin(roll)="<<sin(roll)<<endl;
        // cout<<"roll="<<roll<<endl;
    }
    if(times==3)
    {
        if(presentTime>=0&&presentTime<=T*0.3)
        {
            ftsPos(2,0)=(ftsPos(2,2)+5)*(ftsPos(2,2)+5)/500-53;
            ftsPos(2,1)=ftsPos(2,2)*ftsPos(2,2)/500+38;         
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*28;  
        }
        if(presentTime>T*0.3&&presentTime<=T*0.7)
        {
            ftsPos(2,0)=ftsPos(2,0)+timePeriod*3;
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*6;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*30;
        }
        if(presentTime>T*0.7&&presentTime<=T)
        {
            ftsPos(2,0)=ftsPos(2,0)+timePeriod*2;
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*8;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*12;
        }
    }
    if(times==4)
    {
        if(presentTime>=0&&presentTime<=T*0.3)
        {
            ftsPos(0,0)=-(ftsPos(0,2)+5)*(ftsPos(0,2)+5)/500+52;
            ftsPos(0,1)=ftsPos(0,2)*ftsPos(0,2)/400+43;   
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*28;        
        }
        if(presentTime>0.3*T&&presentTime<=0.7*T)
        {
            ftsPos(0,0)=ftsPos(0,0)-timePeriod*2;
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*6;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*30;
        }
        if(presentTime>0.7*T&&presentTime<=T)
        {
            ftsPos(0,0)=ftsPos(0,0)-timePeriod;
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*6;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*10;
        }
    }

    if(times==5)
    {
        if(presentTime>=0&&presentTime<=T)
        {
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*2;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*3;
            ftsPos(0,1)=ftsPos(0,1)-timePeriod*2;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*3;
            ftsPos(1,1)=ftsPos(1,1)-timePeriod*3;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*3;
            ftsPos(3,1)=ftsPos(3,1)-timePeriod*3;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*3;
        }
    }
    if(times==6)
    {
        if(presentTime>=0&&presentTime<T/4)
        {
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*12;
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*15;
        }
         if(presentTime>=T/4&&presentTime<T/2)
        {
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*7;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*12;
        }
        if(presentTime>=T/2&&presentTime<T*0.75)
        {
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*12;
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*15;
        }
        if(presentTime>=T*0.75&&presentTime<T)
        {
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*7;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*12;
        }
    }
    if(times==7)
    {
        if(presentTime>=0&&presentTime<T/2)
        {
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*8;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*4;
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*4;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*8;
            ftsPos(1,1)=ftsPos(1,1)-timePeriod*2;
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*5;
            ftsPos(3,1)=ftsPos(3,1)-timePeriod*2;
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*5;
        }
        if(presentTime>=T/2&&presentTime<T*0.75)
        {
            ftsPos(0,1)=ftsPos(0,1)-timePeriod*10;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*15;
        }
        if(presentTime>=T*0.75&&presentTime<=T)
        {
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*16;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*9;
        }
    }
    if(times==8)
    {
        if(presentTime>=0&&presentTime<=T/4)
        {
            ftsPos(2,0)=ftsPos(2,0)-timePeriod;
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*10;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*15;
        }
        if(presentTime>T/4&&presentTime<=T/2)
        {
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*15;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*9;
        }
        // float roll=0.5;
        // Matrix<float, 4, 3>targetPos;
        // targetPos(1,1)=-L1*cos(roll)-L3*sin(roll);
        // targetPos(1,2)=-L3*cos(roll)+L1*sin(roll);
        // targetPos(3,1)=-L1*cos(roll)-L3*sin(roll);
        // targetPos(3,2)=-L3*cos(roll)+L1*sin(roll);
        if(presentTime>T/2&&presentTime<=T)
        {
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*6;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*6;
            ftsPos(0,1)=ftsPos(0,1)-timePeriod*5;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*5;
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*(-13); 
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*17;
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*(-13);
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*17;
        }
    }
    //     if(presentTime>T/4&&presentTime<=T/2)
    //     {
    //         ftsPos(2,1)=ftsPos(2,1)-timePeriod*4;
    //         ftsPos(2,2)=ftsPos(2,2)-timePeriod*5;
    //         ftsPos(0,1)=ftsPos(0,1)-timePeriod*3;
    //         ftsPos(0,2)=ftsPos(0,2)-timePeriod*4;
    //         // ftsPos(1,1)=ftsPos(1,1)+timePeriod*(targetPos(1,1)-temp(1,1))/T*4; 
    //         // ftsPos(1,2)=ftsPos(1,2)+timePeriod*(targetPos(1,2)-temp(1,2))/T*4;
    //         // ftsPos(3,1)=ftsPos(3,1)+timePeriod*(targetPos(3,1)-temp(3,1))/T*4;
    //         // ftsPos(3,2)=ftsPos(3,2)+timePeriod*(targetPos(3,2)-temp(3,2))/T*4;
    //         ftsPos(1,1)=ftsPos(1,1)+timePeriod*(-17); 
    //         ftsPos(1,2)=ftsPos(1,2)+timePeriod*20;
    //         ftsPos(3,1)=ftsPos(3,1)+timePeriod*(-17);
    //         ftsPos(3,2)=ftsPos(3,2)+timePeriod*20;
    //     }
    if(times==9)
    {
        if(presentTime>=0&&presentTime<T/4)
        {
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*10;
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*10;
        }
        if(presentTime>=T/4&&presentTime<T/2)
        {
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*7;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*18;
        }
        if(presentTime>T/2&&presentTime<T*0.75)
        {
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*10;
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*10;
        }
        if(presentTime>T*0.75&&presentTime<T)
        {
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*6;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*19;
        }
    }
    if(times==10)
    {
        if(presentTime>=0&&presentTime<T/4)
        {
            ftsPos(0,0)=ftsPos(0,0)+timePeriod;
            ftsPos(0,1)=ftsPos(0,1)-timePeriod*4;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*12;
        }
        if(presentTime>=T/4&&presentTime<T/2)
        {
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*12;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod;
        }
        if(presentTime>=T/2&&presentTime<T*0.75)
        {
            ftsPos(2,0)=ftsPos(2,0)+timePeriod*2;
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*6;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*9;
        }
        if(presentTime>=T*0.75&&presentTime<=T)
        {
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*18;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*2;
        }
    }
    if(times==11)
    {
        ftsPos(0,1)=ftsPos(0,1)-timePeriod*2;
        ftsPos(0,2)=ftsPos(0,2)-timePeriod*5;
        ftsPos(2,1)=ftsPos(2,1)-timePeriod*2;
        ftsPos(2,2)=ftsPos(2,2)-timePeriod*5;  
        ftsPos(1,1)=ftsPos(1,1)-timePeriod*3;
        ftsPos(1,2)=ftsPos(1,2)+timePeriod*3;
        ftsPos(3,1)=ftsPos(3,1)-timePeriod*3;
        ftsPos(3,2)=ftsPos(3,2)+timePeriod*3;
    }
    if(times==12)
    {
        if(presentTime>0&&presentTime<=T/4)
        {    
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*12;
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*4;
        }
        if(presentTime>T/4&&presentTime<=T/2)
        {    
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*8;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*17;
        }
        if(presentTime>=T/2&&presentTime<=T*0.75)
        {
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*8;
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*5;
        }
        if(presentTime>T*0.75&&presentTime<T)
        {    
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*8;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*19;
        }
    }
    if(times==13)
    {
        if(presentTime>=0&&presentTime<=T/4)
        {
            ftsPos(0,1)=ftsPos(0,1)-timePeriod;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*12;
        }
        if(presentTime>T/4&&presentTime<T/2)
        {    
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*12;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*5;
        }
        if(presentTime>T/2&&presentTime<=T*0.75)
        {    
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*4;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*10;
        }
        if(presentTime>T*0.75&&presentTime<=T)
        {    
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*12;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*6;
        }
    }
    if(times==14)
    {
        if(presentTime>0&&presentTime<=T/2)
        {    
            ftsPos(0,1)=ftsPos(0,1)-timePeriod*5;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*5;
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*5;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*5;
            ftsPos(1,1)=ftsPos(1,1)-timePeriod*6;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*5;
            ftsPos(3,1)=ftsPos(3,1)-timePeriod*6;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*5;
        }
        if(presentTime>=T/2&&presentTime<=T*0.75)
        {
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*7;
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*8;
        }
        if(presentTime>T*0.75&&presentTime<T)
        {    
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*7;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*15;
        }
    }
    if(times==15)
    {
        if(presentTime>=0&&presentTime<=T/4)
        {
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*7;
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*8;
        }
        if(presentTime>T/4&&presentTime<T/2)
        {    
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*7;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*15;
        }
        if(presentTime>=T/2&&presentTime<T*0.75)
        {
            ftsPos(0,1)=ftsPos(0,1)-timePeriod;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*14;
        }
        if(presentTime>=T*0.75&&presentTime<T)
        {
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*16;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*7;
        }
    }
    if(times==16)
    {   
        if(presentTime>=0&&presentTime<T/4)
        {
            ftsPos(2,1)=ftsPos(2,1)-timePeriod;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*14;
        }
        if(presentTime>=T/4&&presentTime<=T/2)
        {
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*16;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*7;
        }
        if(presentTime>=T/2&&presentTime<=T)
        {
            ftsPos(0,1)=ftsPos(0,1)-timePeriod*6;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*11;
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*6;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*11;
            ftsPos(1,1)=ftsPos(1,1)-timePeriod*6;
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*11;
            ftsPos(3,1)=ftsPos(3,1)-timePeriod*6;
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*11;
        }     
    }
    if(times==17)
    { 
        if(presentTime>0&&presentTime<T/4)
        {    
            ftsPos(0,1)=ftsPos(0,1)-timePeriod;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*15;
        }
        if(presentTime>T/4&&presentTime<T/2)
        {    
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*8;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*10;
        }
        if(presentTime>T/2&&presentTime<T*0.75)
        {    
            ftsPos(2,1)=ftsPos(2,1)-timePeriod;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*15;
        }
        if(presentTime>T*0.75&&presentTime<T)
        {    
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*8;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*10;
        }
    }
    if(times==18)
    {
        if(presentTime>0&&presentTime<=T/2)
        {    
            ftsPos(0,1)=ftsPos(0,1)-timePeriod*5;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*11;
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*5;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*11;
            ftsPos(1,1)=ftsPos(1,1)-timePeriod;
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*15;
            ftsPos(3,1)=ftsPos(3,1)-timePeriod;
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*15;
        }
        if(presentTime>T/2&&presentTime<T*0.75)
        {    
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*11;
           // ftsPos(1,2)=ftsPos(1,2)-timePeriod;
        }
        if(presentTime>T*0.75&&presentTime<T)
        {    
            ftsPos(1,1)=ftsPos(1,1)-timePeriod*5;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*18;
        }
    }
    if(times==19)
    {
        if(presentTime>0&&presentTime<T/4)
        {    
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*11;
           // ftsPos(3,2)=ftsPos(3,2)-timePeriod;
        }
        if(presentTime>T/4&&presentTime<T/2)
        {    
            ftsPos(3,1)=ftsPos(3,1)-timePeriod*6;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*18;
        }
        if(presentTime>T/2&&presentTime<=T*0.75)
        {    
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*10;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*15;
        }
        if(presentTime>T*0.75&&presentTime<=T)
        {    
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*7;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*12;
        }
    }
    if(times==20)
    {
        if(presentTime>=0&&presentTime<T/4)
        {
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*10;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*15;
        }
        if(presentTime>=T/4&&presentTime<T/2)
        {
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*7;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*12;
        }
        if(presentTime>=T/2&&presentTime<T)
        {
            ftsPos(0,1)=ftsPos(0,1)-timePeriod*6;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*10;
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*6;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*10;
            ftsPos(1,1)=ftsPos(1,1)-timePeriod*2;
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*12;
            ftsPos(3,1)=ftsPos(3,1)-timePeriod*2;
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*12;
        }
    }
    if(times==21)
    { 
        if(presentTime>=0&&presentTime<T/4)
        {    
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*8;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*12;
        }
        if(presentTime>=T/4&&presentTime<T/2)
        {    
            ftsPos(1,1)=ftsPos(1,1)-timePeriod*3;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*10;
        }
        if(presentTime>=T/2&&presentTime<T*0.75)
        {
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*8;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*12;
        }
        if(presentTime>=T*0.75&&presentTime<T)
        {
            ftsPos(3,1)=ftsPos(3,1)-timePeriod*4;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*10;
        }
    }
    if(times==22)
    {
        if(presentTime>=0&&presentTime<T/4)
        {
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*6;
            ftsPos(0,2)=ftsPos(0,2)+timePeriod*14;
        }
        if(presentTime>=T/4&&presentTime<T/2)
        {
            ftsPos(0,1)=ftsPos(0,1)+timePeriod*8;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*12;
        }
        if(presentTime>=T/2&&presentTime<T*0.75)
        {
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*6;
            ftsPos(2,2)=ftsPos(2,2)+timePeriod*13;
        }
        if(presentTime>=T*0.75&&presentTime<T)
        {
            ftsPos(2,1)=ftsPos(2,1)+timePeriod*8;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*12;
        }
    }
    if(times==23)
    {
        if(presentTime>=0&&presentTime<T/2)
        {
            ftsPos(0,1)=ftsPos(0,1)-timePeriod*7;
            ftsPos(0,2)=ftsPos(0,2)-timePeriod*2;
            ftsPos(2,1)=ftsPos(2,1)-timePeriod*7;
            ftsPos(2,2)=ftsPos(2,2)-timePeriod*2;
            ftsPos(1,1)=ftsPos(1,1)-timePeriod*2;
            ftsPos(1,2)=ftsPos(1,2)+timePeriod*10;
            ftsPos(3,1)=ftsPos(3,1)-timePeriod*2;
            ftsPos(3,2)=ftsPos(3,2)+timePeriod*10;
        }
    }
    if(times==24)
    {
        if(presentTime>=0&&presentTime<=T*0.3)
        {
            ftsPos(1,0)=-(ftsPos(1,2)*ftsPos(1,2))/300+52;
            ftsPos(1,1)=-ftsPos(1,2)*ftsPos(1,2)/150-53;  
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*20;        
        }
        if(presentTime>T*0.3&&presentTime<=T)
        {
            ftsPos(1,0)=ftsPos(1,0)-timePeriod;
            ftsPos(1,1)=ftsPos(1,1)+timePeriod*2;
            ftsPos(1,2)=ftsPos(1,2)-timePeriod*20;
        }
    }
    if(times==25)
    {
        if(presentTime>=0&&presentTime<=T*0.3)
        {
            ftsPos(3,0)=ftsPos(3,2)*ftsPos(3,2)/300-52;
            ftsPos(3,1)=-ftsPos(3,2)*ftsPos(3,2)/150-54;  
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*20;        
        }
        if(presentTime>T*0.3&&presentTime<=T)
        {
            ftsPos(3,0)=ftsPos(3,0)+timePeriod;
            ftsPos(3,1)=ftsPos(3,1)+timePeriod*2;
            ftsPos(3,2)=ftsPos(3,2)-timePeriod*19;
        }
    }
    if(times==26)
    {
        targetPos(0,0)=targetPos(1,0)=L2;
        targetPos(2,0)=targetPos(3,0)=-L2;
        targetPos(0,1)=targetPos(2,1)=L1;
        targetPos(1,1)=targetPos(3,1)=-L1;
        targetPos(0,2)=targetPos(1,2)=targetPos(2,2)=targetPos(3,2)=-L3;
        if(presentTime>=0&&presentTime<T/8)ftsPos(0,2)=ftsPos(0,2)+timePeriod*20;
        if(presentTime>T/8-timePeriod/2&&presentTime<T/8+timePeriod/2)
        {
            for(int j=0; j<3; j++){
                temp(0,j)=ftsPos(0,j);
                tempDiff(0,j)=targetPos(0,j)-temp(0,j); 
            }
        }
        if(presentTime>=T/8&&presentTime<T/4)
        {
            ftsPos(0,2)=ftsPos(0,2)+2*timePeriod*tempDiff(0,2);
            ftsPos(0,1)=ftsPos(0,1)+2*timePeriod*tempDiff(0,1);
            ftsPos(0,0)=ftsPos(0,0)+2*timePeriod*tempDiff(0,0);
        }

        if(presentTime>=T/4&&presentTime<T*0.375)ftsPos(2,2)=ftsPos(2,2)+timePeriod*20;
        if(presentTime>T*0.375-timePeriod/2&&presentTime<T*0.375+timePeriod/2)
        {
            for(int j=0; j<3; j++){
                temp(2,j)=ftsPos(2,j);
                tempDiff(2,j)=targetPos(2,j)-temp(2,j); 
            }
        }
        if(presentTime>=T*0.375&&presentTime<T/2)
        {
            ftsPos(2,2)=ftsPos(2,2)+2*timePeriod*tempDiff(2,2);
            ftsPos(2,1)=ftsPos(2,1)+2*timePeriod*tempDiff(2,1);
            ftsPos(2,0)=ftsPos(2,0)+2*timePeriod*tempDiff(2,0);
        }

        if(presentTime>=T/2&&presentTime<T*0.625)ftsPos(1,2)=ftsPos(1,2)+timePeriod*20;
        if(presentTime>T*0.625-timePeriod/2&&presentTime<T*0.625+timePeriod/2)
        {
            for(int j=0; j<3; j++){
                temp(1,j)=ftsPos(1,j);
                tempDiff(1,j)=targetPos(1,j)-temp(1,j); 
            }
        }
        if(presentTime>=T*0.625&&presentTime<T*0.75)
        {
            ftsPos(1,2)=ftsPos(1,2)+2*timePeriod*tempDiff(1,2);
            ftsPos(1,1)=ftsPos(1,1)+2*timePeriod*tempDiff(1,1);
            ftsPos(1,0)=ftsPos(1,0)+2*timePeriod*tempDiff(1,0);
        }

        if(presentTime>=T*0.75&&presentTime<T*0.875)ftsPos(3,2)=ftsPos(3,2)+timePeriod*20;
        if(presentTime>T*0.875-timePeriod/2&&presentTime<T*0.875+timePeriod/2)
        {
            for(int j=0; j<3; j++){
                temp(3,j)=ftsPos(3,j);
                tempDiff(3,j)=targetPos(3,j)-temp(3,j); 
            }
        }
        if(presentTime>=T*0.875&&presentTime<T)
        {
            ftsPos(3,2)=ftsPos(3,2)+2*timePeriod*tempDiff(3,2);
            ftsPos(3,1)=ftsPos(3,1)+2*timePeriod*tempDiff(3,1);
            ftsPos(3,0)=ftsPos(3,0)+2*timePeriod*tempDiff(3,0);
        }
        // for(u_int8_t legNum=0; legNum<4; legNum++)
        // {
        //     if(presentTime>=legNum*T/4+T/8-timePeriod*1.5&&presentTime<legNum*T/4-T/8-timePeriod/2)
        //     {              
        //         for(int j=0; j<3; j++){
        //             temp(legNum,j)=ftsPos(legNum,j);
        //             cout<<"temp("<<legNum<<","<<j<<")="<<temp(legNum,j)<<endl;
        //         }
        //     }
        //     if(presentTime>=legNum*T/4+T/8-timePeriod/2&&presentTime<legNum*T/4+T/8+timePeriod/2)
        //     {
        //             for(int j=0; j<3; j++){

        //                 tempDiff(legNum,j)=targetPos(legNum,j)-temp(legNum,j);
        //                 cout<<"tempDiff("<<legNum<<","<<j<<")="<<tempDiff(legNum,j)<<endl;
        //             }
        //     }
        //     if(presentTime>=legNum*T/4&&presentTime<legNum*T/4+T/8)
        //     {
        //         ftsPos(legNum,2)=ftsPos(legNum,2)+timePeriod*10;
        //     }
        //     if(presentTime>=legNum*T/4+T/8&&presentTime<legNum*T/4+T/4)
        //     {
        //         ftsPos(legNum,2)=ftsPos(legNum,2)+2*timePeriod*tempDiff(legNum,2);
        //         ftsPos(legNum,1)=ftsPos(legNum,1)+2*timePeriod*tempDiff(legNum,1);
        //         ftsPos(legNum,0)=ftsPos(legNum,0)+2*timePeriod*tempDiff(legNum,0);
        //     }
        // }
    }
        
    presentTime = presentTime + timePeriod;
    if(abs(presentTime - timeGait - timePeriod) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        presentTime = 0.0;
        times++;
    }
}
void CreepMotionControl::turn()
{
   // roll = roll + 0.001;
    for(uint8_t legNum=0; legNum<4; legNum++){
        if(legNum==0||legNum==2){       
        ftsPos(legNum,2) = ftsPos(legNum,2)-width/2*sin(roll);
        ftsPos(legNum,1) = ftsPos(legNum,1)+width/2*(1-cos(roll));}
        if(legNum==1||legNum==3){
        ftsPos(legNum,2) = ftsPos(legNum,2)+width/2*sin(roll);
        ftsPos(legNum,1) = ftsPos(legNum,1)-width/2*(1-cos(roll));}
    }
    // for(int i=0; i<4; i++){
    //   for(int j=0; j<3; j++){
    //     cout<<"turnftsPos("<<i<<","<<j<<")="<<ftsPos(i,j)<<endl;
    //   }
    // }  
    cout<<"sin(roll)="<<sin(roll)<<endl;
    cout<<"roll="<<roll<<endl;
    presentTime = presentTime + timePeriod;
    if (abs(presentTime - timeGait - timePeriod) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        presentTime = 0.0;
        times++;
    }
}
void CreepMotionControl::inverseKinematics()  //ftsPos -> motorPos
{   
    // ftsPos(0,0)=L2+20;    // ftsPos(0,1)=L1;    // ftsPos(0,2)=-L3;    // ftsPos(1,0)=L2;    // ftsPos(1,1)=-L1;    // ftsPos(1,2)=-L3;    // ftsPos(2,0)=-L2;    // ftsPos(2,1)=L1;    // ftsPos(2,2)=-L3;    // ftsPos(3,0)=-L2;    // ftsPos(3,1)=-L1;
    // ftsPos(3,2)=-L3;
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
        motorPos(legNum,1) = -factor_xc * (asin(L3 / sqrt( ftsPos(legNum,2)*ftsPos(legNum,2) + ftsPos(legNum,1)*ftsPos(legNum,1) )) + atan2(ftsPos(legNum,2),factor_y * ftsPos(legNum,1)) );     
        motorPos(legNum,0) = -factor_yc * (asin((ftsPos(legNum,1) * ftsPos(legNum,1) + ftsPos(legNum,0) * ftsPos(legNum,0) + ftsPos(legNum,2) * ftsPos(legNum,2) + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (ftsPos(legNum,1) * ftsPos(legNum,1) +  ftsPos(legNum,0) * ftsPos(legNum,0) + ftsPos(legNum,2) * ftsPos(legNum,2) - L3 * L3)))
								- atan2(sqrt(ftsPos(legNum,1) * ftsPos(legNum,1) + ftsPos(legNum,2) * ftsPos(legNum,2) - L3 * L3) , factor_x * ftsPos(legNum,0)));
        motorPos(legNum,2) = -factor_zc * asin((L1 * L1 + L2 * L2 + L3 * L3 - ftsPos(legNum,1) * ftsPos(legNum,1) - ftsPos(legNum,0) * ftsPos(legNum,0) - ftsPos(legNum,2) * ftsPos(legNum,2)) / (2 * L1 * L2));
    }
    for(int i=0; i<4; i++)
    {
        servoPos(i,2)=motorPos(i,2);
    }
    servoPos(0,0)=motorPos(0,0)+motorPos(0,1);
    servoPos(0,1)=motorPos(0,0)-motorPos(0,1);
    servoPos(1,0)=motorPos(1,0)+motorPos(1,1);
    servoPos(1,1)=motorPos(1,0)-motorPos(1,1);
    servoPos(2,0)=motorPos(2,0)-motorPos(2,1);
    servoPos(2,1)=motorPos(2,0)+motorPos(2,1);
    servoPos(3,0)=motorPos(3,0)-motorPos(3,1);
    servoPos(3,1)=motorPos(3,0)+motorPos(3,1);
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            servoPos(i,j)=servoPos(i,j)*180/PI;
        }
    }
}
void CreepMotionControl::turnback(){
    for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
    {
        if(legNum==0||legNum==2){
            ftsPos(legNum,2) = ftsPos(legNum,2)+width/2*sin(roll);
            ftsPos(legNum,1) = ftsPos(legNum,1)-width/2*(1-cos(roll));}
        if(legNum==1||legNum==3){
            ftsPos(legNum,2) = ftsPos(legNum,2)-width/2*sin(roll);
            ftsPos(legNum,1) = ftsPos(legNum,1)+width/2*(1-cos(roll));}
    }
}
void CreepMotionControl::setJointPosition()
{
    int joints = 0;
    for(int i = 0; i<4; i++)
    {
        for(int j = 0; j<3; j++)
        {
            jointPos[joints] = motorPos(i,j);
            //jointPos[joints] = 20/(180/PI);
            //cout<<"jointPos[joints]"<<jointPos[joints]<<endl;
            joints++;
        }
    }
    for (int i = 0; i < 12; i++)
    {
        Tor[i]->setPosition(jointPos[i]);
    }
}

void CreepMotionControl::sensorUpdate()
{
    int motor=0;
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            jointPresentPos(i,j) = Ps[motor]->getValue();
            motor++;
        }
    }
    //cout << "jointPresentPos = "<< jointPresentPos(0,0) <<endl;
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            motorPosFdb(i,j)=jointPresentPos(i,j);
        }
    }
    for (int i=0; i<4; i++)
    {
      Ts[i]->enable(TIME_STEP);   
    }
    // float temp_touch1 = Ts[0]->getValue();
    // float temp_touch2 = Ts[1]->getValue();
    // float temp_touch3 = Ts[2]->getValue();
    // float temp_touch4 = Ts[3]->getValue();
   // for (int i = 0; i < 3; i++)
   // {
       // imu_num(i) = imu->getRollPItchYaw()[i];
   // } 
   // cout << "imu:   "<< imu_num.transpose()<<endl;       
   // for (int i = 0; i < 12; i++)
   // {
       // jointPresentVel(i) = (jointPresentPos(i) - jointLastPos(i))/ timePeriod;
   // }      
   // for (int i = 0; i < 3; i++)
   // {
       // imuVel(i) = (imu_num(i) - Lastimu_num(i)) / timePeriod;
   // } 
}
void CreepMotionControl::forwardKinematics()
{
    ftsPos_bymotor(0,0) = L1*cos(motorPosFdb(0,1))*cos(motorPosFdb(0,0) + PI/2) - L2*(sin(motorPosFdb(0,0) + PI/2)*sin(motorPosFdb(0,2) - PI/2) - cos(motorPosFdb(0,1))*cos(motorPosFdb(0,0) + PI/2)*cos(motorPosFdb(0,2) - PI/2)) + L3*cos(motorPosFdb(0,0) + PI/2)*sin(motorPosFdb(0,1));
    ftsPos_bymotor(0,1) = L2*(cos(motorPosFdb(0,0) + PI/2)*sin(motorPosFdb(0,2) - PI/2) + cos(motorPosFdb(0,1))*cos(motorPosFdb(0,2) - PI/2)*sin(motorPosFdb(0,1) + PI/2)) + L1*cos(motorPosFdb(0,1))*sin(motorPosFdb(0,0) + PI/2) + L3*sin(motorPosFdb(0,1))*sin(motorPosFdb(0,0) + PI/2);
    ftsPos_bymotor(0,2) = L1*sin(motorPosFdb(0,1)) - L3*cos(motorPosFdb(0,1)) + L2*cos(motorPosFdb(0,2) - PI/2)*sin(motorPosFdb(0,1));
    ftsPos_bymotor(1,0) = L1*cos(motorPosFdb(1,1))*cos(motorPosFdb(1,0) - PI/2) - L2*(sin(motorPosFdb(1,0) - PI/2)*sin(motorPosFdb(1,2) + PI/2) - cos(motorPosFdb(1,1))*cos(motorPosFdb(1,0) - PI/2)*cos(motorPosFdb(1,2) + PI/2)) - L3*cos(motorPosFdb(1,0) - PI/2)*sin(motorPosFdb(1,1));
    ftsPos_bymotor(1,1) = L2*(cos(motorPosFdb(1,0) - PI/2)*sin(motorPosFdb(1,2) + PI/2) + cos(motorPosFdb(1,1))*cos(motorPosFdb(1,2) + PI/2)*sin(motorPosFdb(1,0) - PI/2)) + L1*cos(motorPosFdb(1,1))*sin(motorPosFdb(1,0) - PI/2) - L3*sin(motorPosFdb(1,1))*sin(motorPosFdb(0,0) - PI/2);
    ftsPos_bymotor(1,2) = - L3*cos(motorPosFdb(1,1)) - L1*sin(motorPosFdb(1,1)) - L2*cos(motorPosFdb(1,2) + PI/2)*sin(motorPosFdb(1,1));
    ftsPos_bymotor(2,0) = L1*cos(motorPosFdb(2,1))*cos(motorPosFdb(2,0) + PI/2) - L2*(sin(motorPosFdb(2,0) + PI/2)*sin(motorPosFdb(2,2) + PI/2) - cos(motorPosFdb(2,1))*cos(motorPosFdb(2,0) + PI/2)*cos(motorPosFdb(2,2) + PI/2)) + L3*cos(motorPosFdb(2,0) + PI/2)*sin(motorPosFdb(0,1));
    ftsPos_bymotor(2,1) = L2*(cos(motorPosFdb(2,0) + PI/2)*sin(motorPosFdb(2,2) + PI/2) + cos(motorPosFdb(2,1))*cos(motorPosFdb(2,2) + PI/2)*sin(motorPosFdb(2,0) + PI/2)) + L1*cos(motorPosFdb(2,1))*sin(motorPosFdb(2,0) + PI/2) + L3*sin(motorPosFdb(2,1))*sin(motorPosFdb(0,0) + PI/2);
    ftsPos_bymotor(2,2) = L1*sin(motorPosFdb(2,1)) - L3*cos(motorPosFdb(2,1)) + L2*cos(motorPosFdb(2,2) + PI/2)*sin(motorPosFdb(2,1));
    ftsPos_bymotor(3,0) = L1*cos(motorPosFdb(3,1))*cos(motorPosFdb(3,0) - PI/2) - L2*(sin(motorPosFdb(3,0) - PI/2)*sin(motorPosFdb(3,2) - PI/2) - cos(motorPosFdb(3,1))*cos(motorPosFdb(3,0) - PI/2)*cos(motorPosFdb(3,2) - PI/2)) - L3*cos(motorPosFdb(3,0) - PI/2)*sin(motorPosFdb(0,1));
    ftsPos_bymotor(3,1) = L2*(cos(motorPosFdb(3,0) - PI/2)*sin(motorPosFdb(3,2) - PI/2) + cos(motorPosFdb(3,1))*cos(motorPosFdb(3,2) - PI/2)*sin(motorPosFdb(3,0) - PI/2)) + L1*cos(motorPosFdb(3,2))*sin(motorPosFdb(3,0) - PI/2) - L3*sin(motorPosFdb(3,1))*sin(motorPosFdb(0,0) - PI/2);
    ftsPos_bymotor(3,2) = - L3*cos(motorPosFdb(3,1)) - L1*sin(motorPosFdb(3,1)) - L2*cos(motorPosFdb(3,2) - PI/2)*sin(motorPosFdb(3,1));
    for(uint8_t legNum=0; legNum<4; legNum++)
    {      
        for(int i=0; i<3; i++)
        {
            shoulderPos_bymotor.row(legNum)(i) = footPos(legNum,i) - ftsPos_bymotor(legNum,i);
        }
        shoulderPos_bymotor.row(legNum)(3) = 1; 
        for(int i=0; i<4; i++)
        {
            comPos_bymotor(i) = (shoulderPos_bymotor.row(0)(i) + shoulderPos_bymotor.row(2)(i))/2; // 电机反馈的质心坐标（在世界坐标系中）X Y Z alpha               
        } 
    }
}
    /*        
    Vector<float, 3> swingphaseVelocity =  (stancePhaseEndPos.row(legNum) - stancePhaseStartPos.row(legNum)) / (timeGait - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0)) - timePeriod);
    // float Vx = swingphaseVelocity(0);
    // float Vy = swingphaseVelocity(1);
    // float Vz = swingphaseVelocity(2);
    float onestepH = 10;
    float onestepL = stancePhaseEndPos(legNum,0) - stancePhaseStartPos(legNum,0);
    if(abs(stancePhaseEndPos(legNum,0) - stancePhaseStartPos(legNum,0))<1e-3){
    onestepL=1e-4;}
    //for(uint8_t pos; pos<3; pos++){
    ftsPos(legNum, 0) = ftsPos(legNum, 0) - swingphaseVelocity(0) * timePeriod;
    footPos(legNum,0) = ftsPos(legNum,0) + shoulderPos(legNum,0) + comPos(0);
    cout<<"swingphaseVelocity(0)="<<swingphaseVelocity(0)<<endl;
    // stanceFlag(legNum) = false;
    if(legNum==0||legNum==1){
    ftsPos(legNum, 2) = -4*onestepH/(onestepL*onestepL)*(ftsPos(legNum, 0)-L2-onestepL/2)*(ftsPos(legNum, 0)-L2-onestepL/2)+onestepH  - swingphaseVelocity(2) * timePeriod -L3;} // Y's direction movement
    if(legNum==2||legNum==3){
    ftsPos(legNum, 2) = -4*onestepH/(onestepL*onestepL)*(ftsPos(legNum, 0)+L2-onestepL/2)*(ftsPos(legNum, 0)+L2-onestepL/2)+onestepH  - swingphaseVelocity(2) * timePeriod -L3;} // Y's direction movement
    */ // trajectory of quadratic function
