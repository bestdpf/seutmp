
#ifndef SEXP_PARSER_H
#define SEXP_PARSER_H

#include <sfsexp/sexp.h>
#include <string>
#include <boost/lexical_cast.hpp>
#include "math/Math.hpp"

namespace parser {
	
class SexpParser
{
public:
	SexpParser()
	{
    }

    ~SexpParser()
    {
    }
	
	/** 分析sExp单元中的值/parse the value of the sExp atom 
	*  @param [in] sexp sExp单元/sExp atom pointer
	*  @param [out] res 解析出的值/the value
	*  @return 如有任何异常返回FALSE,否则返回TRUE/if there is anything wrong FALSE, else TRUE
	*/
	static bool parseValue( const sexp_t* sexp, std::string& res )
	{
		if ( sexp && sexp->ty == SEXP_VALUE )
		{
			res = sexp->val;
			return true;
		}
		return false;
	}
	
	/** 分析sExp单元中的值/parse the value of the sExp atom 
	*  @param [in] sexp sExp单元/sExp atom pointer
	*  @param [out] res 解析出的值/the value
	*  @return 如有任何异常返回FALSE,否则返回TRUE/if there is anything wrong FALSE, else TRUE
	*/
	static bool parseValue( const sexp_t* sexp, char &res )
	{
		if ( sexp && sexp->ty == SEXP_VALUE )
		{
			res = sexp->val[0];
			return true;
		}
		return false;
	}

	/** 分析sExp单元中的值/parse the value of the sExp atom 
	*  @param [in] sexp sExp单元/sExp atom pointer
	*  @param [out] res 解析出的值/the value
	*  @return 如有任何异常返回FALSE,否则返回TRUE/if there is anything wrong FALSE, else TRUE
	*/
	static bool parseValue( const sexp_t* sexp, int &res )
	{
		if ( sexp && sexp->ty == SEXP_VALUE )
		{
			res = atoi(sexp->val);
			return true;
		}
		return false;
	}
	
	/** 分析sExp单元中的值/parse the value of the sExp atom 
	*  @param [in] sexp sExp单元/sExp atom pointer
	*  @param [out] res 解析出的值/the value
	*  @return 如有任何异常返回FALSE,否则返回TRUE/if there is anything wrong FALSE, else TRUE
	*/
	static bool parseValue( const sexp_t* sexp, unsigned int &res )
	{
		if ( sexp && sexp->ty == SEXP_VALUE )
		{
			res = static_cast<unsigned int>( atoi(sexp->val) );
			return true;
		}
		return false;
	}
	
	/** 分析sExp单元中的值/parse the value of the sExp atom 
	*  @param [in] sexp sExp单元/Exp atom pointer
	*  @param [out] res 解析出的值/the value
	*  @return 如有任何异常返回FALSE,否则返回TRUE/if there is anything wrong FALSE, else TRUE
	*/
	static bool parseValue( const sexp_t* sexp, float &res )
	{
		if ( sexp && sexp->ty == SEXP_VALUE )
		{
			res = atof(sexp->val);
			return true;
		}
		return false;
	}

    /** 分析sExp单元中的值/parse the value of the sExp atom 
	*  @param [in] sexp sExp单元/Exp atom pointer
	*  @param [out] res 解析出的值/the value
	*  @return 如有任何异常返回FALSE,否则返回TRUE/if there is anything wrong FALSE, else TRUE
	*/
	static bool parseValue( const sexp_t* sexp, bool &res )
	{
		if ( sexp && sexp->ty == SEXP_VALUE )
		{
			res = ( 't' == *(sexp->val) || 'T' == *(sexp->val) );
			return true;
		}
		return false;
	}
		
	/** 分析sExp单元中的值/parse the value of the sExp atom 
	*  @param [in] sexp sExp单元/sExp atom pointer
	*  @param [out] res 解析出的值/the value
	*  @return 如有任何异常返回FALSE,否则返回TRUE/if there is anything wrong FALSE, else TRUE
	*/
    template <typename DATATYPE, size_t ELEMENTS>
	static bool parseValue( const sexp_t* sexp, math::TVector<DATATYPE,ELEMENTS> &res )
	{
		for ( size_t i=0;i<ELEMENTS;i++)
		{
			if ( 0==sexp || sexp->ty != SEXP_VALUE )
			{
				return false;
			}
			res[i] = boost::lexical_cast<DATATYPE>(sexp->val);
			sexp=sexp->next;
		}
		return true;
	}
	
	/** 分析sExp单元中的值/parse the value of the sExp atom 
	*  @param [in] sexp sExp单元/sExp atom pointer
	*  @param [out] res 解析出的值/the value
	*  @return 如有任何异常返回FALSE,否则返回TRUE/if there is anything wrong FALSE, else TRUE
	*/
	static bool parseValue( const sexp_t* sexp, math::TransMatrixf &res )
	{
		for ( int i=0;i<4;i++ )
		{
			for( int j=0;j<4;j++ )
			{
				if ( 0==sexp || sexp->ty != SEXP_VALUE )
				{
					return false;
				}
                if ( j<3 ) res[i][j] = atof(sexp->val);
				sexp=sexp->next;
			}
		}
		return true;
	}
	
	/** 分析sExp列表中的值/parse the given value of the sExp list 
	*  @param [in] sexp sExp列表/sExp list pointer
	*  @param [in] given 指定的名称/the name of given value
	*  @param [out] res 解析出的值/the value
	*  @return 如果该列表值名称与指定名称相同,且解析成功返回TRUE,否则返回FALSE/if the value name is the same with given name, and successfully get the value, return TRUE, otherwise return FALSE
	*/
	template <class T>
	static bool parseGivenValue( const sexp_t* sexp, const char* given, T &res )
	{
		if ( sexp && sexp->ty == SEXP_LIST )
		{
			const sexp_t* tmp = sexp->list;
			if ( tmp->ty == SEXP_VALUE )
			{
				if ( 0==strcmp(given,tmp->val) )
				{
					return parseValue(tmp->next,res);
				}
			}
			return false;
		}
		return false;
	}

    template <class T>
	static bool parseGivenValue( const sexp_t* sexp, const std::string& given, T &res )
        {
            return parseGivenValue<T>( sexp, given.c_str(), res );
        }
    
    
	/** 分析sExp单元中的值/parse a array value of the sExp atom 
	*  @param [in] sexp sExp单元/Exp atom pointer
	*  @param [in] size the size of array
	*  @param [out] res 解析出的值/the value
	*  @return 如有任何异常返回FALSE,否则返回TRUE/if there is anything wrong FALSE, else TRUE
	*/
	static bool parseArrayValue( const sexp_t* sexp, unsigned int size, float *res )
	{
		for( unsigned int i=0; i<size; i++ )
		{
			if ( !sexp || sexp->ty != SEXP_VALUE ) return false;
			res[i] = atof(sexp->val);
			sexp = sexp->next;
		}
		return true;
	}
	
	static bool isVal(const sexp_t* sexp)
	{
	   return SEXP_VALUE == sexp->ty;
	}
	
	static bool isList(const sexp_t* sexp)
	{
	   return SEXP_LIST == sexp->ty;
	}
};

} // namespace parser

#endif // SEXP_PARSER_H
