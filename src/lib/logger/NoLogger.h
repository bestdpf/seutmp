/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: NoLogger.h 1806 2008-06-27 11:46:13Z xy $
 *
 ****************************************************************************/

///////////////// DEFINES TO AVOID COPY ////////////////////
///////////////// NOTE: ALL EMPTY //////////////////////////
#undef DECLARE_GRAPHIC_LOGGER
#define DECLARE_GRAPHIC_LOGGER

#undef DECLARE_STATIC_GRAPHIC_LOGGER
#define DECLARE_STATIC_GRAPHIC_LOGGER

#undef DEFINE_STATIC_GRAPHIC_LOGGER
#define DEFINE_STATIC_GRAPHIC_LOGGER(CLASS_NAME)

#undef LOG
#define LOG(...)

#undef BEGIN_ADD_LOG_LAYER
#define BEGIN_ADD_LOG_LAYER(CLASS_NAME)

#undef BEGIN_ADD_STATIC_LOG_LAYER
#define BEGIN_ADD_STATIC_LOG_LAYER(CLASS_NAME)

#undef ADD_LOG_LAYER
#define ADD_LOG_LAYER(LAYER_NAME)

#undef END_ADD_LOG_LAYER
#define END_ADD_LOG_LAYER(CLASS_NAME)

#undef END_ADD_STATIC_LOG_LAYER
#define END_ADD_STATIC_LOG_LAYER(CLASS_NAME)

#undef LOG_FLUSH
#define LOG_FLUSH

#undef LOG_PRINT
#define LOG_PRINT(layer,text)

#undef LOG_PRINTF
#define LOG_PRINTF(layer,...)

#undef LOG_PRINT_VECTOR2
#define LOG_PRINT_VECTOR2(layer,v)

#undef LOG_PRINT_VECTOR3
#define LOG_PRINT_VECTOR3(lyaer,v)

#undef LOG_PRINT_MATRIX_3X3
#define LOG_PRINT_MATRIX_3X3(layer,m)

#undef LOG_PRINT_TRANS_MATRIX
#define LOG_PRINT_TRANS_MATRIX(layer,t)

#undef LOG_BOX
#define LOG_BOX(layer,trans,size,color)

#undef LOG_RED_BOX
#define LOG_RED_BOX(layer,trans,size)

#undef LOG_BLUE_BOX
#define LOG_BLUE_BOX(layer,trans,size)

#undef LOG_YELLOW_BOX
#define LOG_YELLOW_BOX(layer,trans,size)

// sphere
#undef LOG_RED_SPHERE
#define LOG_RED_SPHERE(layer,pos,radius)

#undef LOG_GREEN_SPHERE
#define LOG_GREEN_SPHERE(layer,pos,radius)

#undef LOG_BLUE_SPHERE
#define LOG_BLUE_SPHERE(layer,pos,radius)

#undef LOG_YELLOW_SPHERE
#define LOG_YELLOW_SPHERE(layer,pos,radius)

#undef LOG_LINE
#define LOG_LINE(layer,p0,p1,color)

#undef LOG_BLUE_LINE
#define LOG_BLUE_LINE(layer,p0,p1)

#undef LOG_RED_LINE
#define LOG_RED_LINE(layer,p0,p1)

// cylinder
#undef LOG_YELLOW_CYLINDER
#define LOG_YELLOW_CYLINDER(layer,pos,radius)

// 2D line
#undef LOG_LINE_2D
#define LOG_LINE_2D(layer,p0,p1,color)

#undef LOG_RED_LINE_2D
#define LOG_RED_LINE_2D(layer,p0,p1)

#undef LOG_GREEN_LINE_2D
#define LOG_GREEN_LINE_2D(layer,p0,p1)

#undef LOG_BLUE_LINE_2D
#define LOG_BLUE_LINE_2D(layer,p0,p1)

#undef LOG_YELLOW_LINE_2D
#define LOG_YELLOW_LINE_2D(layer,p0,p1)

// axes
#undef LOG_AXES
#define LOG_AXES(layer,trans,size)
