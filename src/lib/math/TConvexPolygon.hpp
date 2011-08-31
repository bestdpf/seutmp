/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

/**
 * @file   TPolygon.hpp
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Sat Oct 13 04:30:48 2007
 * 
 * @brief  the convex polygon template class
 * 
 * 
 */



#ifndef MATH_TCONVEX_POLYGON_HPP
#define MATH_TCONVEX_POLYGON_HPP

#include <list>
#include "TSegment2.hpp"
#include "Angle.hpp"

namespace math{
    
    template<typename DATATYPE>
    class TConvexPolygon
    {
    public:
        typedef std::list< TSegment2<DATATYPE> > TEdges;
        
        TConvexPolygon()
            {
            }
        
        // triangle
        TConvexPolygon(const TVector<DATATYPE,2>& p1,
                 const TVector<DATATYPE,2>& p2,
                 const TVector<DATATYPE,2>& p3)
            {
                create(p1,p2,p3);
            }
        
        void create(const TVector<DATATYPE,2>& p1,
                    const TVector<DATATYPE,2>& p2,
                    const TVector<DATATYPE,2>& p3)
            {
                mEdges.clear();
                mEdges.push_back( TSegment2<DATATYPE>(p1,p2));
                mEdges.push_back( TSegment2<DATATYPE>(p2,p3));
                mEdges.push_back( TSegment2<DATATYPE>(p3,p1));
            }
        
        // quadrangle
        TConvexPolygon(const TVector<DATATYPE,2>& p1,
                       const TVector<DATATYPE,2>& p2,
                       const TVector<DATATYPE,2>& p3,
                       const TVector<DATATYPE,2>& p4)
            {
                create(p1,p2,p3,p4);
            }
        
        void create(const TVector<DATATYPE,2>& p1,
                    const TVector<DATATYPE,2>& p2,
                    const TVector<DATATYPE,2>& p3,
                    const TVector<DATATYPE,2>& p4)
            {
                mEdges.clear();
                mEdges.push_back( TSegment2<DATATYPE>(p1,p2));
                mEdges.push_back( TSegment2<DATATYPE>(p2,p3));
                mEdges.push_back( TSegment2<DATATYPE>(p3,p4));
                mEdges.push_back( TSegment2<DATATYPE>(p4,p1));
            }

        // more than 4 edges
        TConvexPolygon( const std::vector< TVector<DATATYPE,2> >& p )
            {
                create(p);
            }
        
        void create(const std::vector< TVector<DATATYPE,2> >& p )
            {
                mEdges.clear();
                typename TEdges::const_iterator begin = p.begin();
                typename TEdges::const_iterator end = p.end();
                typename TEdges::const_iterator iter1 = begin;
                typename TEdges::const_iterator iter2 = begin;
                for (  ++iter1; iter2!=end; ++iter1,++iter2 ){
                    mEdges.push_back(TSegment2<DATATYPE>(*iter1,*iter2));
                }
                mEdges.push_back(TSegment2<DATATYPE>(*iter1,*begin));
            }

        /** 
         * check if a given point is inside the polygon
         * 
         * @param p the given point
         * 
         * @return boolen indicates the point is inside the polygon
         */
        bool isInside(const TVector<DATATYPE,2>& p)const
            {
                if ( size()< 3 ) return false; // the polygon is not exist
                
                // check carefully
                // make a line which through the point p
                typename TEdges::const_iterator iter=mEdges.begin();
                TVector<DATATYPE,2> p0 = (iter->p0() + iter->p1())*0.5;
                TLine2<DATATYPE> line(p,p0);
                // check the intersection number of the line and polygon
                // if there are 2 intersections,
                // the check point p in segement of intersections
                // ==> inside the polygon
                TVector<DATATYPE,2> p1;
                for( ++iter; iter!=mEdges.end(); ++iter ){
                    if ( iter->calIntersection(line,p1) ){
                        if ( p[0]>=std::min(p0[0],p1[0])
                             && p[0]<=std::max(p0[0],p1[0])
                             && p[1]>=std::min(p0[1],p1[1])
                             && p[1]<=std::max(p0[1],p1[1])){
                            return true;
                        }
                    }
                }
                return false;
            }

        struct AABB
        {
            TVector<DATATYPE,2> max;
            TVector<DATATYPE,2> min;
        };

        AABB calAABB() const
            {
                AABB aabb;
                if ( mEdges.empty() ) return aabb;
                
                typename TEdges::const_iterator iter=mEdges.begin();
                const TVector<DATATYPE,2>& p = iter->p0();
                DATATYPE maxX=p.x();
                DATATYPE maxY=p.y();
                DATATYPE minX=p.x();
                DATATYPE minY=p.y();
                for( ++iter; iter!=mEdges.end(); ++iter ){
                    const TVector<DATATYPE,2>& p = iter->p0();
                    DATATYPE x = p.x();
                    DATATYPE y = p.y();
                    if ( x > maxX ){
                        maxX = x;
                    }
                    else if ( x < minX ){
                        minX = x;
                    }
                    if ( y > maxY ){
                        maxY = y;
                    }
                    else if ( y < minY ){
                        minY = y;
                    }
                }
                aabb.max.x() = maxX;
                aabb.max.y() = maxY;
                aabb.min.x() = minX;
                aabb.min.y() = minY;
                return aabb;
            }

        bool isInAABB( const TVector<DATATYPE,2> p, const AABB& aabb ) const
            {
                return p.x()<=aabb.max.x() && p.y()<=aabb.max.y()
                    && p.x()>=aabb.min.x() && p.y()>=aabb.min.y();
            }
        
        /** 
         * calculate the centroid of the convex polygon
         * 
         * 
         * @return the centroid point
         */
        TVector<DATATYPE,2> calCentroid()const
            {
                if ( size() < 3 ){
                    std::cerr<<__FILE__<<__FUNCTION__<<__LINE__
                             <<"size error"<<std::endl;
                    return mEdges.begin()->p0();
                }

                typename TEdges::const_iterator begin = mEdges.begin();
                typename TEdges::const_iterator end = mEdges.end();
                typename TEdges::const_iterator iter(begin);

                const TVector<DATATYPE,2>& a = begin->p0();
                TVector<DATATYPE,2> centroid(0,0);
                DATATYPE total = 0;
                for( ++iter,--end; iter!=end; ++iter ){
                    // (a.x*b.y-a.y*b.x+b.x*c.y-c.x*b.y+c.x*a.y-a.x*c.y)/2.0;
                    const TVector<DATATYPE,2>& b = iter->p0();
                    const TVector<DATATYPE,2>& c = iter->p1();
                    DATATYPE area =
                        (a[0]*b[1]-a[1]*b[0]+b[0]*c[1]
                         -c[0]*b[1]+c[0]*a[1]-a[0]*c[1]);
                    centroid += ((a+b+c)*area);
                    total += area;
                }
                total*=3.0f;
                centroid/=total;

                return centroid;
            }

        /** 
         * check intersections with other convex polygon
         * 
         * @param cp the other convex polygon
         * @param iss the vector of intersections ( vertex of polygon )
         * 
         * @return is the two convex polygon intersecting
         */
        bool contain(const TConvexPolygon& cp,
                       std::vector< TVector<DATATYPE,2> >& iss )
            {
                iss.clear();

                AABB aabb = calAABB();
                
                for( typename TEdges::const_iterator iter=cp.begin();
                     iter!=cp.end(); ++iter ){
                    const TVector<DATATYPE,2>& p = iter->p0();
                    if ( isInAABB( p, aabb) ){
                        if ( isInside( p ) ){
                            iss.push_back( p );
                        }
                    }
                }
                return !iss.empty();
            }

        /** 
         * check intersections with other convex polygon
         * this function is a little faster than get the
         * vertex in the other polygon ( i.e. the other same named method)
         * 
         * @param cp the other convex polygon
         * 
         * @return is the two convex polygon intersecting
         */
        bool contain(const TConvexPolygon& cp)
            {
                AABB aabb = calAABB();
                for( typename TEdges::const_iterator iter=cp.begin();
                     iter!=cp.end(); ++iter ){
                    const TVector<DATATYPE,2>& p = iter->p0();
                    if ( isInAABB( p, aabb) ){
                        if ( isInside( p ) ){
                            return true;
                        }
                    }
                }
                return false;
            }
        
   
        typename TEdges::const_iterator begin()const{ return mEdges.begin(); }
        typename TEdges::const_iterator end()  const{ return mEdges.end();   }
        size_t size() const { return mEdges.size(); }

        /** 
         * create a rectangle from its center, direction and size
         * 
         * @param c the center of the rectangle
         * @param ang the direction of the rectangle
         * @param size the size of the rectangle
         */
        void createRectangle(const TVector<DATATYPE,2>& c, AngDeg ang,
                             const TVector<DATATYPE,2>& size)
            {
                DATATYPE dist = size.length()*0.5;
                
                AngDeg ang1 = atan2Deg(size.y(), size.x());
                AngDeg ang2 = normalizeAngle( 180 - ang1 );
                AngDeg ang3 = normalizeAngle( 180 + ang1 );
                AngDeg ang4 = -ang1;

                ang1 = normalizeAngle( ang1 + ang );
                ang2 = normalizeAngle( ang2 + ang );
                ang3 = normalizeAngle( ang3 + ang );
                ang4 = normalizeAngle( ang4 + ang );
                
                TVector<DATATYPE,2> p1
                    = c + pol2xyz(TVector<DATATYPE,2>(dist,ang1));
                TVector<DATATYPE,2> p2
                    = c + pol2xyz(TVector<DATATYPE,2>(dist,ang2));
                TVector<DATATYPE,2> p3
                    = c + pol2xyz(TVector<DATATYPE,2>(dist,ang3));
                TVector<DATATYPE,2> p4
                    = c + pol2xyz(TVector<DATATYPE,2>(dist,ang4));
                create(p1,p2,p3,p4);
            }
        
    protected:
        TEdges mEdges;
    };

    template <typename DATATYPE>
    class TVoronoi : public TConvexPolygon<DATATYPE>
    {
    public:
        TVoronoi()
            {
            }

        ~TVoronoi()
            {
            }

        /** set a point as the "inner point",
         *  which will be used while cutting the Voronoi polygon by a line
         *  keeping the "inner point" always inside the Voronoi polygon
         *  @param[in] p the "inner point" be setted
         *  @return boolen indicats if setting is OK
         */
        bool setInnerPoint(const TVector<DATATYPE,2>& p)
 	        {
                if ( isInside(p) )
                {
                    mInnerPoint = p;
                    return true;
                }

                return false;
            }

        void cutByLine(const TLine2<DATATYPE>& line)
 	        {
                typename TConvexPolygon<DATATYPE>::TEdges::iterator iter,intersectEdge[2];
                TVector<DATATYPE,2> intersection[2];

                unsigned int intersectionNum = 0;
                for( iter = this->mEdges.begin();
                     iter!= this->mEdges.end()&&intersectionNum<2;
                     ++iter )
                {
                    if ( iter->calIntersection(line,intersection[intersectionNum]) )
                    {
                        intersectEdge[intersectionNum] = iter;
                        intersectionNum++;
                    }
                }
                
                if ( 2 == intersectionNum )
                {
                    int innerLocation = line.location(mInnerPoint);
                    //ASSERT(innerLocation!=0);
                    if ( innerLocation==0 )
                    {
                        std::cerr<<__FILE__<<__LINE__<<__FUNCTION__
                                 <<"the line can not though the inner point!"<<std::endl;
                        return;
                    }

                    int vertexLocaltion = line.location(intersectEdge[0]->p0());
                    if ( 0==vertexLocaltion )
                        vertexLocaltion = line.location(intersectEdge[0]->p1());
                    // ASSERT(0!=vertexLocaltion);
                    if ( vertexLocaltion != innerLocation )
                    {
                        TVector<DATATYPE,2> p0 = intersection[0];
                        TVector<DATATYPE,2> p1 = intersectEdge[0]->p1();
                        TVector<DATATYPE,2> p2 = intersectEdge[1]->p0();
                        TVector<DATATYPE,2> p3 = intersection[1];
                        this->mEdges.erase(this->mEdges.begin(),++intersectEdge[0]);
                        this->mEdges.erase(intersectEdge[1],this->mEdges.end());
                        
                        this->mEdges.push_front(TSegment2<DATATYPE>(p0,p1));
                        /*if ( p2 != p3 )*/ this->mEdges.push_back(TSegment2<DATATYPE>(p2,p3));
                        /*if ( p3 != p0 )*/ this->mEdges.push_back(TSegment2<DATATYPE>(p3,p0));
                        if ( this->size()<3 )
                            std::cerr<<__FILE__<<__LINE__<<__FUNCTION__
                                     <<"size = "<<this->size()<<std::endl;
                    }
                    else
                    {
                        TVector<DATATYPE,2> p0 = intersectEdge[0]->p0();
                        TVector<DATATYPE,2> p1 = intersection[0];
                        TVector<DATATYPE,2> p2 = intersection[1];
                        TVector<DATATYPE,2> p3 = intersectEdge[1]->p1();
                        typename TConvexPolygon<DATATYPE>::TEdges::iterator insert = this->mEdges.erase(intersectEdge[0],++intersectEdge[1]);

                        /*if ( p2 != p3 )*/ insert = this->mEdges.insert(insert,TSegment2<DATATYPE>(p2,p3));
                        /*if ( p1 != p2 )*/ insert = this->mEdges.insert(insert,TSegment2<DATATYPE>(p1,p2));
                        /*if ( p0 != p1 )*/ insert = this->mEdges.insert(insert,TSegment2<DATATYPE>(p0,p1));
                        if ( this->size()<3 )
                            std::cerr<<__FILE__<<__LINE__<<__FUNCTION__
                                     <<"size = "<<this->size()<<std::endl;
                    }
                }
                //else
                //{
                //      cout<<"intersectionNum="<<intersectionNum<<endl;
                //      if ( 1 == intersectionNum ) cout<<"the only one is "<< intersection[0] <<endl;
                //}
            }
        
    private:
        TVector<DATATYPE,2> mInnerPoint;
    };
    
} // namespace math

#endif // MATH_TCONVEX_POLYGON_HPP
