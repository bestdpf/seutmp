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
 * @file   AStar.hpp
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Tue Oct 23 06:37:54 2007
 * 
 * @brief  this file implement the A* search by template
 *
 * the STATE is the state type which should contain functions:
 * - moreCost: the estimation of future cost
 * - cost: the summary of costed and mreoCost
 * - sameAs: whether the two STATE are the same
 * - operator<: the set will sorted by the operator<,
 *   usually it defined as "return cost() < r.cost()"
 *
 * generate_next_states: is a boost functor which accept start STATE
 * and target STATE as input, generates the next possible STATE,
 * which returned by the third parameter.
 */


#ifndef A_STAR_H
#define A_STAR_H

#include <list>
#include <set>
#include <vector>
#include <boost/function.hpp>
//#include <iostream>

template<typename STATE>
std::list<STATE> AStar(const STATE& start, const STATE& target,
                       boost::function<void (const STATE&, const STATE&,
                                             const STATE&,
                                             std::vector<STATE>&,
                                             std::vector<STATE>&)>
                       generate_next_states)
{
    //std::cout<<"start A* ------------------"<<std::endl;
    std::set<STATE> open;
    std::set<STATE> closed;

    STATE current = start;
    while ( current.moreCost() > 0.0000001f ){
        //std::cout<<"state: "<<current.p()<<std::endl;
        typename std::set<STATE>::iterator iter = closed.begin();
        for(; closed.end() != iter; ++iter){
            if ( iter->sameAs(current) ) break;
        }
        if ( closed.end() == iter ){
            // insert to closed
            closed.insert(current);
        }

        iter = open.begin();
        for(; open.end() != iter; ++iter){
            if (iter->sameAs(current)){
                open.erase(iter);
                break;
            }
        }
            
        std::vector<STATE> nextStates, unReach;
        generate_next_states(current, target, target, nextStates, unReach);
        //std::cout<<"next size: "<<nextStates.size()<<std::endl;
        for( typename std::vector<STATE>::iterator iter=nextStates.begin();
             nextStates.end()!=iter; ++iter){
                
            typename std::set<STATE>::iterator citer = closed.begin();
            for(; citer != closed.end(); ++citer){
                if ( citer->sameAs(*iter) ) break;
            }
                
            if ( closed.end() != citer ){
                //std::cout<<"the state "<<iter->p()
                //<<"have been closed"<<std::endl;
                continue; // the state have be closed
            }

            iter->setPrevious(current);
                
            typename std::set<STATE>::iterator oiter = open.begin();
            for(; oiter != open.end(); ++oiter){
                if ( oiter->sameAs(*iter) ) break;
            }
                
            if ( open.end() == oiter ){
                //std::cout<<"insert "<<iter->p()
                //<<" ("<<iter->cost()<<")"<<std::endl;
                open.insert(*iter); // first reach this state
            }
            else{
                if ( oiter->cost() > iter->cost() ){
                    //std::cout<<"refresh "<<iter->p()
                    //<<" "<<oiter->cost()<<" -> "
                        //<<iter->cost()<<std::endl;
                    // find a better path to reach this state
                    open.erase(oiter);
                    open.insert(*iter);
                }
            }
        }

        if ( open.empty() ){
            //std::cout<<"no path! open empty"<<std::endl;
            break; // no path!
        }

        // start from the best state at currently
        current = *open.begin();
    }

    std::list<STATE> path;
    while ( !start.sameAs(current) )
    {
        path.push_front(current);
        STATE previous = current.previous();
        typename std::set<STATE>::const_iterator p = closed.begin();
        for(; p!=closed.end(); ++p ){
            if ( previous.sameAs(*p) ) break;
        }
        if ( closed.end() == p ){
            //std::cout<<"no path! closed empty"<<std::endl;
            // no path: closed empty!
            break;
        }
        current = *p;
    }

    //std::cout<<"end A* ----------------"<<std::endl;
    return path;
}

#endif // A_STAR_H
