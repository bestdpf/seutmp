/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include <iostream>
//#include "../../../config.h"
#include "Options.h"

namespace configuration {

    Options::Options() {
        mGenOpts = boost::shared_ptr<po::options_description > (new po::options_description("Generic Options"));
        mGenOpts->add_options()
                ("help,h",
                "Print this message and exit.")
                ("server,s",
                po::value<std::string > ()->default_value("localhost"),
                "Specify the host of server.")
                ("port,p",
                po::value<int>()->default_value(3100),
                "Specify the port number of server.")
                ("teamname,t",
                po::value<std::string > ()->default_value("SEU"),
                "Specify the teamname of agent.")
                ("unum,u",
                po::value<unsigned int>()->default_value(0),
                "Specify the unum of agent.")
                ("formation_conf,f",
                po::value<std::string > ()->default_value("formation.conf"),
                "Specify the formation config file.")
                ("offclient,o",
                "offclient mode switch.")
                ;

        mHiddenOpts = boost::shared_ptr<po::options_description > (new po::options_description("Hidden Options"));
        //mHiddenOpts->add_options()
        //("demo",po::value<std::string>(),"Specify what you want the robot to do.")
        //;

        mPosOpts = boost::shared_ptr<po::positional_options_description > (new po::positional_options_description);
        //mPosOpts->add("demo",-1);

        mCmdOpts = boost::shared_ptr<po::options_description > (new po::options_description());
        mCmdOpts->add(*mGenOpts).add(*mHiddenOpts);

        mVisibleOpts = boost::shared_ptr<po::options_description > (new po::options_description("seu-spark"" Options"));
        mVisibleOpts->add(*mGenOpts);
    }

    void Options::parseCmdLine(int argc, char* argv[]) {
        try {
            //store(po::parse_command_line(argc,argv,*mCmdOpts),mVm);
            po::store(po::command_line_parser(argc, argv).options(*mCmdOpts).positional(*mPosOpts).run(), mVm);
            po::notify(mVm);

            if (mVm.count("help")) {
                std::cout << *mVisibleOpts;
                exit(0);
            }
        } catch (std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }

    void Options::greetings() const {
        std::cout <<
                "--------------------------------------------------------------------------\n"
                "                                                               #          \n"
                "      ###                                                    ###          \n"
                "       ##                                                    ######       \n"
                "       ##              #                  #                  ######       \n"
                "      ####             ####               #                # ######       \n"
                "     #####           ######               #              # ########       \n"
                "     #########        ###                 #              ##########       \n"
                "     #########        ##    ####          ##    #        ##########       \n"
                "    ##########        ## #######          ##  ####       ###########      \n"
                "   ## ########         # #########        #########       ############    \n"
                "   ##########         ######   ##     ############         ###########    \n"
                "   ########       #  ######   ##        ######         #   ##   #####     \n"
                "   #######        ## #####    ##         ###          ###     ###         \n"
                "     #####        ## ######   ##         ###          ##     ####         \n"
                "      ###         ##  #####   ##         ##   ##             ####         \n"
                "     #### ###     ##  ####   ##         ###    ##             ##          \n"
                "   ######  ###     #   ###  ###         ###    ###            ###         \n"
                "   # ####   ##          #   ###       #####     ###         #######       \n"
                "     ####               #              ###       #         ########       \n"
                "      ##                                                   #  ###         \n"
                "                                                            ####          \n"
                "\n"
                "--------------------------------------------------------------------------\n"
                "  RoboCup 3D Simulation Team, Southeast University, China   \n"
                "  Research Coordinator: Tan Yingzi                          \n"
                "  Team Coordinator:     Xu Yingqiu                          \n"
                "  Team Members:\n"
                "  2005.   Xu Yuan, Shi Chang'e             \n"
                "  2006.   Xu Yuan, Jiang Chunlu, Wang Qiang\n"
                "  2007.   Xu Yuan, Zhao Xuqing, Chen Si, Jiang Hong\n"
                "          Sheng Hui, Qian Chen, Chen Tianhao, Xu Xiaoli\n"
                "  2008.   Xu Yuan, Chen Si, Hu Ming, Huang Ru, Yu Honglin\n"
                "  2009.   Zhou Yanjun, Chen Si, Yi Yimin\n"
                "  2010.   Zhao Bolu, Wang Sitan, Guo XiaXia, Yu Xiapfan, etc.\n"
                "  2011.   Zhao Bolu, Wang Sitan, Zhong Li, Li Ge, Chen Liang, Zhao Yue, Duan Pengfei, Wang Chenyang, You Weiwei,etc.\n"
                "  All rights reserved, see README for more information.     \n"
                "--------------------------------------------------------------------------\n"
                "  Version: ""seu-spark-iran2010""\n"
                "  Build Time: "__DATE__" -- "__TIME__"\n"
                "--------------------------------------------------------------------------\n"
                << std::endl;
    }

} // namespace configuration
