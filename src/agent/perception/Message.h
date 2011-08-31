/* 
 * File:   Message.h
 * Author: root
 *
 * Created on 2009年12月8日, 上午8:48
 */

#ifndef _MESSAGE_H
#define	_MESSAGE_H
#include "configuration/Configuration.h"
#include<iostream>
#include<string>
using namespace std;
#include<stdio.h>
#define MYTEAMS 'S'
#define SEN1  'A'    //句型1
#define DRCT  'D'    // direct 指挥句型
#define ADJUS  64
struct mVector3f{
    float x;
    float y;
    float z;
};
/*
 *message 格式
 *  ++++5++++1++++5++++2
 * 第0位为队伍判断
 *  1  为应接收的队员号
 *  2  表示句型
 *  3－20 内容
 */
/*句型1
 *  3  队员是否倒 0表示倒，1或其他表示未倒
 *  4,5,6, 球的X坐标，A,B...表示负
 *  7，8，9 球的Y坐标
 *  10，11，12 人的X坐标
 *  13，14，15 人的Y坐标
 */
class Message {
public:
    Message();
    Message(char *);
    Message(const Message& orig);
    virtual ~Message();
    bool DestructMessage(char *);
    char* makeMessage(char* p_targ,int p_recvNum,bool p_isFall,mVector3f p_ballPos, mVector3f p_roboPos);
bool DestructSen1(const char *);
        char* makeSen1(char* p_targ,int p_recvNum,bool p_isFall,mVector3f p_ballPos, mVector3f p_roboPos);

        bool DestructDirectMsg(const char * );
        std::string MakeDirectMsg( configuration::Formation::FormationType p_ft,int p_passZoneX,int p_passZoneY,int p_FastestNum);
configuration::Formation::FormationType getFormation(){return mFT;}
int getZoneX(){return mpassZoneX;}
int getZoneY(){return mpassZoneY;}
unsigned int getFastestNum(){return mFastestNum;}
private:
    char* mMsg;
    bool mIsMyTeam;
    int mRecvNum;
    bool mIsFall;
    mVector3f mBallPos;
    mVector3f mRoboPos;
    configuration::Formation::FormationType mFT;
    int mpassZoneX;
    int mpassZoneY;
    unsigned int mFastestNum;

    
};

#endif	/* _MESSAGE_H */

