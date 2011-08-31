/* 
 * File:   Message.cpp
 * Author: root
 * 
 * Created on 2009年12月8日, 上午8:48
 */

#include "Message.h"
#include<stdio.h>
#include<stdlib.h>
Message::Message() {
     mFT= configuration::Formation::FT_HOME;
     mpassZoneX=-1;
     mpassZoneY=-1;
     mFastestNum=0;
}

Message::Message(const Message& orig) {
}

Message::~Message() {
}

/*句型1
 *  3  队员是否倒 0表示倒，1或其他表示未倒
 *  4,5,6, 球的X坐标，A,B...表示负
 *  7，8，9 球的Y坐标
 *  10，11，12 人的X坐标
 *  13，14，15 人的Y坐标
 */
bool Message::DestructSen1(const char *senten) {
     int q =0;
    while(q < 20){
        if(senten[q]==','){
            break;
        }
        q++;
    }
  //  if(q>=20){
   //     return false;
   // }
    //cout <<q<<endl;
    char* sen =(char*)senten + q+1;
    //cout << sen << endl;
    if (sen[0] != MYTEAMS) {
        mIsMyTeam = false;
        return false;
    }
    if (sen[2] != SEN1) {
        return false;
    }
    mMsg =(char*) sen;
    mRecvNum =((int) sen[1]) - 65;
    mIsFall = (sen[3] == '0') ? true : false;
    float tempf = 0.0;
    if(sen[4]>=65 ){
        tempf = (sen[4]-65)*100+(sen[5]-48)*10+sen[6]-48;
        tempf = -tempf;
    }else{
        tempf = (sen[4]-48)*100+(sen[5]-48)*10+sen[6]-48;
    }
    mBallPos.x =tempf/100;
     tempf = 0.0;
    if(sen[7]>=65 ){
        tempf = (sen[7]-65)*100+(sen[8]-48)*10+sen[9]-48;
        tempf = -tempf;
    }else{
        tempf = (sen[7]-48)*100+(sen[8]-48)*10+sen[9]-48;
    }
    mBallPos.y = tempf/100;
    tempf = 0.0;
    if(sen[10]>=65 ){
        tempf = (sen[10]-65)*100+(sen[11]-48)*10+sen[12]-48;
        tempf = -tempf;
    }else{
        tempf = (sen[10]-48)*100+(sen[11]-48)*10+sen[12]-48;
    }
    mRoboPos.x = tempf/100;
      tempf = 0.0;
    if(sen[13]>=65 ){
        tempf = (sen[13]-65)*100+(sen[14]-48)*10+sen[15]-48;
        tempf = -tempf;
    }else{
        tempf = (sen[13]-48)*100+(sen[14]-48)*10+sen[15]-48;
    }
    mRoboPos.y = tempf/100;
    return true;
}
char* Message::makeSen1(char* p_targ, int p_recvNum, bool p_isFall, mVector3f p_ballPos, mVector3f p_roboPos){
    p_targ = (char*) malloc( 16 * sizeof (char));
    if(p_targ == NULL){
        cout<<"ERROR";
        return p_targ;
    }

    p_targ[0] = MYTEAMS;
    p_targ[1] = (char)(p_recvNum + 65);
    p_targ[2] = SEN1;
    p_targ[3] = (p_isFall)?'0':'1';
    int temp;
    temp = p_ballPos.x*100;
    if(temp<0){
        temp = -temp;
       p_targ[4] =(char)(65+(temp/100)%10);
    }else{
        p_targ[4] =(char)(48+(temp/100)%10);
    }
     p_targ[5] = (char) (48+(temp/10)%10);
       p_targ[6] = (char) (48+(temp)%10);

       temp = p_ballPos.y*100;
       if(temp<0){
        temp = -temp;
       p_targ[7] =(char)(65+(temp/100)%10);
    }else{
        p_targ[7] =(char)(48+(temp/100)%10);
    }
        p_targ[8] = (char) (48+(temp/10)%10);
       p_targ[9] = (char) (48+(temp)%10);
       temp = p_roboPos.x*100;
          if(temp<0){
        temp = -temp;
       p_targ[10] =(char)(65+(temp/100)%10);
    }else{
        p_targ[10] =(char)(48+(temp/100)%10);
    }
        p_targ[11] = (char) (48+(temp/10)%10);
       p_targ[12] = (char) (48+(temp)%10);

       temp = p_roboPos.y*100;
     if(temp<0){
        temp = -temp;
       p_targ[13] =(char)(65+(temp/100)%10);
    }else{
        p_targ[13] =(char)(48+(temp/100)%10);
    }
        p_targ[14] = (char) (48+(temp/10)%10);
       p_targ[15] = (char) (48+(temp)%10);

    return p_targ;
}

/*指挥句型格式
 *  1  MYTEAMS
 *  2  DRCT
 *  3  Formation 阵形数字
 *  4  最佳传球区域X字母表示，左下角起始
 *  5  最佳传球区域Y字母表示，左下角起始
 */
bool Message::DestructDirectMsg(const char * senten)
 {
  //  cout <<"FAST" <<mFastestNum<<endl;
int q =0;
    while(q < 20){
        if(senten[q]==','){
            break;
        }
        q++;
    }

    char* sen =(char*)senten + q+1;
    if (sen[0] != MYTEAMS) {
        mIsMyTeam = false;
        return false;
    }
    if(sen[1]!=DRCT){//不是该句型
        return false;
    }
    int l_ft = sen[2]-'0';

    if(l_ft>=configuration::Formation::FT_HOME && l_ft < configuration::Formation::FT_NULL){
    mFT= (configuration::Formation::FormationType)l_ft;
    }else{
        mFT= configuration::Formation::FT_HOME;
    }
    mpassZoneX = sen[3]-'A';
    mpassZoneY=sen[4]-'A';
    mFastestNum=sen[5]-'A';
   // cout <<"FAST22222222222" <<" " << sen[5]<< " "<< mFastestNum<<endl;
    return true;
    
 }
        std::string Message::MakeDirectMsg(configuration::Formation::FormationType p_ft,int p_passZoneX,int p_passZoneY,int p_FastestNum){
            std::stringstream ss;
            ss << MYTEAMS<< DRCT<<p_ft<<(char)(p_passZoneX+(int)'A')<<(char)(p_passZoneY+(int)'A')<<(char)(p_FastestNum+(int)'A');
            return ss.str();
        }

