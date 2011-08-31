/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef COMMUNIT_HPP
#define COMMUNIT_HPP

/**
 * @file   CommUnit.hpp
 * 
 * @brief  encapsulate the net IO and file IO
 * 
 */

#include "ClassException.hpp"
#include "net/SocketStream.hpp"
#include "Singleton.hpp"
#include <fstream>


using namespace std;
	
class CommUnit:public Singleton<CommUnit>
{
public:
	CommUnit():mNetSS(NULL),mIFSS(NULL)
        {

        }
	
	~CommUnit()
        {
            disconnect();
        }

    /** 
     * connect to the server or open a file
     * 
     * @param address the server name or the file name, it depends on the foreignPort value
     * @param foreignPort if positive, it is the port of server, if minus, means the address is the file name
     */
	void connect(const string address,short foreignPort)
        {
            // close the socket or the fstream firstly
            disconnect();
            
            if(foreignPort>0)					//online mode
            {
                mNetSS = new net::PrefixedSocketStream<net::TCPSocket>(address,foreignPort);
            }
            else								//offline mode
            {
                mIFSS = new ifstream(address.c_str());
                if(!(*mIFSS))
                    throw(ClassException<CommUnit>
                          ("can not open file: "+address));
            }
        }

    /** 
     * close the socket or the fstream
     * 
     */
    void disconnect()
        {
            if ( NULL != mNetSS )
            {
                delete mNetSS;
                mNetSS = NULL;
            }
            if ( NULL != mIFSS )
            {
                delete mIFSS;
                mIFSS = NULL;
            }
        }

    CommUnit& operator >> ( std::string& msg )
        {
            if( NULL != mNetSS )			//online
            {
                (*mNetSS)>>msg;
            }
            else if ( NULL != mIFSS ) //offline
            {
                getline((*mIFSS),msg);
            }
            return *this;
        }
	
	template <class T>
	CommUnit& operator << ( const T& msg )
        {
            if( NULL != mNetSS )			//online
            {
                (*mNetSS)<<msg;
            }
            return *this;
			
        }
	
	void send()
        {
            if ( NULL != mNetSS )		//online
                (*mNetSS)<<net::send;
        }
	
private:
	net::PrefixedSocketStream<net::TCPSocket> *mNetSS;
	ifstream *mIFSS;
};//class CommUnit

#define COMM CommUnit::GetSingleton()

#endif // COMMUNIT_HPP
