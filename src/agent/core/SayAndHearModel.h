/* 
 * File:   SayAndHearModel.h
 * Author: robocup
 *
 * Created on 7/1/2011 14:13
 */

#ifndef SAYANDHEARMODEL_H
#define	SAYANDHEARMODEL_H

#include "core/WorldModel.h"
#include "math/Math.hpp"
#include "Singleton.hpp"
#include <string>
#include<iostream>

namespace core {
    using namespace std;
using namespace core;
    class SayAndHearModel : public Singleton<SayAndHearModel> {
    public:

        SayAndHearModel() : mCycles(0), mCanSay(false), mIsHeard(false), mBase(80) {
            mSayString = "";
        };

        virtual ~SayAndHearModel() {
        };

        //ascii　range 42~126 '*'~'~' 共85个

        void DecToEightyFifth(const int orig, char& high, char& low) {
            //   cout << orig <<endl;
            high = (char) (orig / mBase) + '*';
            //  cout <<"dech"<< (int)high<<endl;
            low = (char) (orig % mBase) + '*';
            // cout <<"decl"<< (int)low<<endl;
        }

        int formatFloat(float orig) {
            int res = std::floor(orig * 100);
            res += (mBase * mBase / 2);
            return res;
        }

        float resolveFloat(int orig) {
            float res = orig - (mBase * mBase / 2);
            res = res / 100.0f;
            return res;
        }

        int formatInt(int orig) {
            return orig + (mBase * mBase / 2);
        }

        int resolveInt(int orig) {

            return orig - (mBase * mBase / 2);
        }

        void EightyFifthToDec(const char origHigh, const char origLow, int & res) {

            res = (origHigh - '*') * mBase + (origLow - '*');
        }
        void update();

        float getHearSendTime() const {
            return mHearSendTime;
        }

        math::Vector2f getHearBallPos() const {
            return mHearBallPos;
        }

        bool IsHearPlayerFallen() const {
            return mHearIsPlayerFallen;
        }

        math::AngDeg getHearPlayerBodyDirection() const {
            return mHearPlayerBodyDirection;
        }

        math::Vector2f getHearPlayerPos() const {
            return mHearPlayerPos;
        }

        unsigned int getHearPlayerID() const {
            return mHearPlayerID;
        }

        float getHearTime() const {
            return mHearTime;
        }

        std::string getSayString() const {
            return mSayString;
        }

        bool IsCanSay() const {
            return mCanSay;
        }

        bool IsHeard() const {
            return mIsHeard;
        }

        void printhear() {

            cout << "mHearString" << mHearString << endl;
            cout << "mHearPlayerID" << mHearPlayerID << endl;
            cout << "mHearPlayerPos" << mHearPlayerPos << endl;
            cout << "mHearPlayerBodyDirection" << mHearPlayerBodyDirection << endl;
            cout << "mHearIsPlayerFallen" << mHearIsPlayerFallen << endl;
            cout << "mHearBallPos" << mHearBallPos << endl;
            cout << "mHearSendTime" << mHearSendTime << endl;
        }

        void printsay() ;




    private:
        void calcCanSay();
        void makeMsg();
        bool resolveMsg();
        const int mBase;


        //say

        int mCycles;
        bool mCanSay;
        std::string mSayString;


        //hear
        bool mIsHeard;
        float mHearTime;
        std::string mHearString;

        unsigned int mHearPlayerID; //1
        math::Vector2f mHearPlayerPos; //2-3 4-5
        math::AngDeg mHearPlayerBodyDirection; //6-7
        bool mHearIsPlayerFallen; //8
        math::Vector2f mHearBallPos; //9-10 11-12
        float mHearSendTime; //13-14
        //     int mState;



    };
}
#define SHM core::SayAndHearModel::GetSingleton()
#endif	/* SAYANDHEARMODEL_H */

