/*****************************************************************************************
 *                                      SEU-3D
 *                     -------------------------------------------------
 * Copyright (c) 2005, Yuan XU<xychn15@yahoo.com.cn>,Chang'e SHI<evelinesce@yahoo.com.cn>
 * Copyright (c) 2006, Yuan XU<xuyuan.cn@gmail.com>,Chunlu JIANG<JamAceWatermelon@gmail.com>
 * Southeast University ,China
 * All rights reserved.
 *
 * $Id: Defines.hpp,v 1.1.1.1 2007/03/07 03:28:10 xy Exp $
 *
 * Additionally,this program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 ****************************************************************************************/

#ifndef _DEFINES_H
#define _DEFINES_H

#ifdef __cplusplus
extern "C"
{
#endif



#ifdef __cplusplus
}
#endif

#include <cmath>       // needed for M_PI constant

namespace math
{

typedef unsigned int size_type;

static const float EPSILON=0.0001f;  // Value used for floating point equality tests.

#ifndef M_PI
#define M_PI 3.14151926535897f
#endif

// some compiler-specific defines
#if             defined(_MSC_VER)

        // other symbols
        #define f_inline                        __forceinline
#elif   defined(__GNUC__)
        #define f_inline                        inline
#endif

}// namespace maths

#endif /* _DEFINES_H */
