
#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

class Logger
{
public:
    static const int debugMode = 0;

    static void showLog(std::string comment, int level = 0, const std::string &file = __FILE__, int line = __LINE__);

    template <typename... Args>
    static void showLogMulComments(int level, const std::string &file, int line, Args &&...args)
    {

        if (debugMode == 0)
        {
            return;
        }

        std::ostringstream oss;
        (oss << ... << args);
        std::string comment = oss.str();

        Logger::showLog(comment, level, file, line);
    }
};
