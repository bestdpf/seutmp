/* 
 * File:   SayAndHearModel.cpp
 * Author: robocup
 * 
 * Created on 7/1/2011
 */

#include "SayAndHearModel.h"
#include<stdlib.h>

#include <cmath>
#include <string>
#include <sstream>
#include <vector>
namespace core {
    using namespace std;
    using namespace boost;

    void SayAndHearModel::update() {
        calcCanSay();
        makeMsg();

        const vector<shared_ptr<perception::Hear> >& hear = WM.lastPerception().hear();
        if (hear.empty()) {
            mIsHeard = false;
            return;
        }
        mIsHeard = false;

        FOR_EACH(iter, hear) {
            mHearString = ((*iter)->message());
            // ;
            mIsHeard = resolveMsg();
            break;
        }

    }

    void SayAndHearModel::calcCanSay() {
        int time = std::floor(WM.getSimTime()*100);
        int myUnum = WM.getMyUnum();
        const int teamNum = 9;
        int mod = time % 180;
        int type = (mod / 4) % teamNum;

        if (myUnum - 1 == type) {
            mCanSay = true;
        } else {
            mCanSay = false;
        }
    }

    void SayAndHearModel::makeMsg() {
        mSayString = "";
        std::stringstream ss;
        int tempInt = 0;
        char tempH;
        char tempL;
        //playerID
        // tempInt = formatInt(WM.getMyUnum());
        //  DecToEightyFifth(tempInt, tempH, tempL);
        ss << (char) (WM.getMyUnum() + '*');
        //player pos x,y
        tempInt = formatFloat(WM.getMyGlobalPos().x());
        DecToEightyFifth(tempInt, tempH, tempL);
        ss << tempH << tempL;

        tempInt = formatFloat(WM.getMyGlobalPos().y());
        DecToEightyFifth(tempInt, tempH, tempL);
        ss << tempH << tempL;
        //player bodydir
        tempInt = formatFloat(WM.getMyBodyDirection() / 10.0f);
        DecToEightyFifth(tempInt, tempH, tempL);
        ss << tempH << tempL;
        //player is fallen
        if (WM.isFall()) {
            ss << '1';
        } else {
            ss << '0';
        }
        //now time
        tempInt = formatFloat(WM.getBallGlobalPos().x());
        DecToEightyFifth(tempInt, tempH, tempL);
        ss << tempH << tempL;
        tempInt = formatFloat(WM.getBallGlobalPos().y());
        DecToEightyFifth(tempInt, tempH, tempL);
        ss << tempH << tempL;

        float time = WM.getSimTime() - std::floor(WM.getSimTime() / 10)*10;
        tempInt = formatFloat(time);
        DecToEightyFifth(tempInt, tempH, tempL);
        ss << tempH << tempL;
        mSayString = ss.str();
    }

    bool SayAndHearModel::resolveMsg() {
        const int offset = 2;
        if (mHearString.size() < offset + 14) {
            return false;
        }
        int tempInt = 0;
        char tempH;
        char tempL;
        tempL = mHearString[offset + 0];

        mHearPlayerID = tempL - '*';

        tempH = mHearString[offset + 1];
        tempL = mHearString[offset + 2];
        EightyFifthToDec(tempH, tempL, tempInt);
        mHearPlayerPos.x() = resolveFloat(tempInt);

        tempH = mHearString[offset + 3];
        tempL = mHearString[offset + 4];
        EightyFifthToDec(tempH, tempL, tempInt);
        mHearPlayerPos.y() = resolveFloat(tempInt);

        tempH = mHearString[offset + 5];
        tempL = mHearString[offset + 6];
        EightyFifthToDec(tempH, tempL, tempInt);
        mHearPlayerBodyDirection = resolveFloat(tempInt)*10.0f;
        ;

        tempH = mHearString[offset + 7];
        if (tempH == '1') {
            mHearIsPlayerFallen = true;
        } else {
            mHearIsPlayerFallen = false;
        }
        tempH = mHearString[offset + 8];
        tempL = mHearString[offset + 9];
        EightyFifthToDec(tempH, tempL, tempInt);
        mHearBallPos.x() = resolveFloat(tempInt);

        tempH = mHearString[offset + 10];
        tempL = mHearString[offset + 11];
        EightyFifthToDec(tempH, tempL, tempInt);
        mHearBallPos.y() = resolveFloat(tempInt);

        tempH = mHearString[offset + 12];
        tempL = mHearString[offset + 13];
        EightyFifthToDec(tempH, tempL, tempInt);
        mHearSendTime = resolveFloat(tempInt);
        return true;

    }

       void SayAndHearModel::printsay() {

//            cout << "mSayString" << mSayString << endl;
//            cout << "mSayPlayerID" << WM.getMyUnum() << endl;
//            cout << "mSayPlayerPos" << WM.getMyGlobalPos2D() << endl;
//            cout << "mSayPlayerBodyDirection" << WM.getMyBodyDirection() << endl;
//            cout << "mSayIsPlayerFallen" << WM.isFall() << endl;
//            cout << "mSayBallPos" << WM.getBallGlobalPos() << endl;
//            cout << "mSaySendTime" << WM.getSimTime() << " " << WM.getSimTime() - std::floor(WM.getSimTime() / 10)*10 << endl;
        }


}
