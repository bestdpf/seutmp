/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _PERCEPTION_PARTICLESFILTER_HPP_
#define _PERCEPTION_PARTICLESFILTER_HPP_

#include <list>
#include <boost/function.hpp>

namespace perception{

    template<typename PARTICLES_TYPE>
    class ParticlesFilter
    {
    public:

        struct Particle
        {
            PARTICLES_TYPE d;   /**< the data */
            float p;            /**< the possibility */
        };
        
        ParticlesFilter(const PARTICLES_TYPE& c, const PARTICLES_TYPE& dist, const PARTICLES_TYPE& size)
            {
                // just for Vector3f now
                PARTICLES_TYPE minP = c-dist;
                float deltaX = dist[0] / size[0] * 2;
                float deltaY = dist[1] / size[1] * 2;
                float deltaZ = dist[2] / size[2] * 2;
                for( int x=0; x<size[0]; x++ ){
                    for( int y=0; y<size[1]; y++ ){
                        for( int z=0; z<size[2]; z++ ){
                            Particle p;
                            p.d = minP+Vector3f(deltaX*x,deltaY*y,deltaZ*z);
                            p.p = 1;
                            mParticles.push_back(p);
                        }
                    }
                }
            }

        void exclude(boost::function<bool (const PARTICLES_TYPE&)> in)
            {
                typename std::list<Particle>::iterator iter=mParticles.begin();
                while ( mParticles.end() != iter ){
                    if ( !in(iter->d) ){
                        iter = mParticles.erase(iter);
                    }
                    else{
                        ++iter;
                    }
                }
            }

        void updatePossibility(boost::function<float (const PARTICLES_TYPE&)> func)
            {
                FOR_EACH( iter, mParticles ){
                    iter->p = func( iter->d );
                }
            }

        bool results(PARTICLES_TYPE& res) const
            {
                if ( mParticles.empty() ) return false;
                float poss = 0;
                FOR_EACH( iter, mParticles ){
                    res += iter->d * iter->p;
                    poss += iter->p;
                }
                if ( 0 == poss ) return false;
                res/=poss;
                return true;
            }
    
    private:
        std::list<Particle> mParticles;
    };
    
} // namespace perception


#endif /* _PERCEPTION_PARTICLESFILTER_HPP_ */
