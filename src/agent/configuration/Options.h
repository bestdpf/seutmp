/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef OPTIONS_H
#define OPTIONS_H

#include <string>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

namespace po = boost::program_options;

namespace configuration
{
    class Options
    {
    public:
        Options();
        ~Options(){}
	
        void parseCmdLine(int argc, char* argv[]);
	
        void greetings() const;
	
        template<typename T>
        T arg(const std::string& opt) const
            {
                return mVm[opt].as<T>();
            }
	
        int count(const std::string& opt) const { return mVm.count(opt); }

    private:
        /* generic options description */
        boost::shared_ptr<po::options_description> mGenOpts;
	
        /* positional options description */
        boost::shared_ptr<po::positional_options_description> mPosOpts;
	
        /* hidden options description */
        boost::shared_ptr<po::options_description> mHiddenOpts;

        /* command line options description */
        boost::shared_ptr<po::options_description> mCmdOpts;

        /* print options description while helping */
        boost::shared_ptr<po::options_description> mVisibleOpts;

        po::variables_map mVm;
    };
}

#endif // OPTIONS_H
