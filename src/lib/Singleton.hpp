/*
 *Copyright (c) 2005, Xu Yuan<xychn15@yahoo.com.cn>,Shi Chang'e<evelinesce@yahoo.com.cn>
 *Southeast University ,China
 *All rights reserved.
 */

/***************************************************************************
 *            Singleton.h
 *
 *	Thanks to Scott Bilas, the original version comes from <<Game Programming Gems>>
 *  2005/09/20		Xu Yuan<xychn15@yahoo.com.cn>		Initial version created
 * LastModify:
 * $Id: Singleton.hpp,v 1.2 2007/03/13 07:30:06 xy Exp $
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
 
#ifndef _SINGLETON_H
#define _SINGLETON_H

#ifdef __cplusplus
extern "C"
{
#endif



#ifdef __cplusplus
}
#endif

#include <cassert>

#ifdef DEBUG
#define ASSERT assert
#else
#define ASSERT(x);
#endif

template <typename T> class Singleton
{
	static T* ms_Singleton;

public:
	~Singleton()
	{
		ASSERT( ms_Singleton );
		ms_Singleton = 0;
	}
	static T& GetSingleton()
	{
		static T the_T;
		ASSERT( ms_Singleton );
		return ( *ms_Singleton );
	}
	/*
	static T* GetSingletonPtr( void )
	{
		ASSERT( ms_Singleton );
		return ( ms_Singleton );
	}*/
protected:
	Singleton()
	{
		assert( !ms_Singleton );
	//	int offset = (int)(T*)1 - (int)(Singleton <T>*)(T*)1;
	//	ms_Singleton = (T*)((int)this + offset );
                ms_Singleton = static_cast<T*>(this);
	}
};

template <typename T> T* Singleton <T>::ms_Singleton = 0;

#ifdef NO_ASSERT
#undef NO_ASSERT
#endif
#undef ASSERT


#endif /* _SINGLETON_H */
