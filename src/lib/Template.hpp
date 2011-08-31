
#ifndef TEMPLATE_HPP
#define TEMPLATE_HPP

#include <map>
#include <algorithm>

template <class TMapFirst, class TMapSecond>
typename std::map<TMapFirst,TMapSecond>::const_iterator findMapBySecondValue( const std::map<TMapFirst,TMapSecond>& m, const TMapSecond s )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter=m.begin();
    for ( ; iter!=m.end(); ++iter )
    {
        if ( s == iter->second ) break;
    }
	return iter;
}

template <class TMapFirst, class TMapSecond>
typename std::map<TMapFirst,TMapSecond>::const_iterator findMapBySecondRef( const std::map<TMapFirst,TMapSecond>& m, const TMapSecond& s )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter=m.begin();
    for ( ; iter!=m.end(); ++iter )
    {
        if ( s == iter->second ) break;
    }
	return iter;
}

/** get the first value by the given value as second in the map,
 *  if can not find, return the last value as default
 */
template <class TMapFirst, class TMapSecond>
TMapFirst getFirstValueBySecondValue( const std::map<TMapFirst,TMapSecond>& m, const TMapSecond s )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter = findMapBySecondValue(m,s);
	if ( m.end() == iter ) --iter;
	return iter->first;
}

/** get the first reference by the given value as second in the map,
 *  if can not find, return the last value as default
 */
template <class TMapFirst, class TMapSecond>
const TMapFirst& getFirstRefBySecondValue( const std::map<TMapFirst,TMapSecond>& m, const TMapSecond s )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter = findMapBySecondValue(m,s);
	if ( m.end() == iter ) --iter;
	return iter->first;
}

/** get the first value by the given reference as second in the map,
 *  if can not find, return the last value as default
 */
template <class TMapFirst, class TMapSecond>
TMapFirst getFirstValueBySecondRef( const std::map<TMapFirst,TMapSecond>& m, const TMapSecond& s )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter = findMapBySecondRef(m,s);
	if ( m.end() == iter ) --iter;
	return iter->first;
}

/** get the first reference by the given reference as second in the map,
 *  if can not find, return the last value as default
 */
template <class TMapFirst, class TMapSecond>
const TMapFirst& getFirstRefBySecondRef( const std::map<TMapFirst,TMapSecond>& m, const TMapSecond& s )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter = findMapBySecondRef(m,s);
	if ( m.end() == iter ) --iter;
	return iter->first;
}

/** get the second value by the given value as first in the map,
 *  if can not find, return the last value as default
 */
template <class TMapFirst, class TMapSecond>
TMapSecond getSecondValueByFirstValue( const std::map<TMapFirst,TMapSecond>& m, const TMapFirst f )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter = m.find(f);
	if ( m.end() == iter ) --iter;
	return iter->second;
}

/** get the second reference by the given value as first in the map,
 *  if can not find, return the last value as default
 */
template <class TMapFirst, class TMapSecond>
const TMapSecond& getSecondRefByFirstValue( const std::map<TMapFirst,TMapSecond>& m, const TMapFirst f )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter = m.find(f);
	if ( m.end() == iter ) --iter;
	return iter->second;
}

/** get the second value by the given reference as first in the map,
 *  if can not find, return the last value as default
 */
template <class TMapFirst, class TMapSecond>
TMapSecond getSecondValueByFirstRef( const std::map<TMapFirst,TMapSecond>& m, const TMapFirst& f )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter = m.find(f);
	if ( m.end() == iter ) --iter;
	return iter->second;
}

/** get the second reference by the given reference as first in the map,
 *  if can not find, return the last value as default
 */
template <class TMapFirst, class TMapSecond>
const TMapSecond& getSecondRefByFirstRef( const std::map<TMapFirst,TMapSecond>& m, const TMapFirst& f )
{
	typename std::map<TMapFirst,TMapSecond>::const_iterator iter = m.find(f);
	if ( m.end() == iter ) --iter;
	return iter->second;
}

/**
 *  Preprosessor macro to define a STL iterator for loop:
 *  FOR_EACH(iterator, container)
 */
#define FOR_EACH(iter,cont) \
    for( __typeof__((cont).begin()) iter = (cont).begin(); \
         iter != (cont).end(); \
         ++iter)

#endif // TEMPLATE_HPP
