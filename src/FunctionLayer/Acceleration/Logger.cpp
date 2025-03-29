#include "Logger.h"

void Logger::showLog(std::string comment, int level, const std::string &file, int line)
{

    if (debugMode == 0)
    {
        return;
    }

    const std::string COLOR_RESET = "\033[0m";
    const std::string COLOR_RED = "\033[31m";
    const std::string COLOR_GREEN = "\033[32m";
    const std::string COLOR_YELLOW = "\033[33m";
    const std::string COLOR_BLUE = "\033[34m";
    const std::string COLOR_MAGENTA = "\033[35m";
    const std::string COLOR_CYAN = "\033[36m";

    std::string color;
    switch (level)
    {
    case 0:
        color = COLOR_CYAN;
        break;
    case 1:
        color = COLOR_GREEN;
        break;
    case 2:
        color = COLOR_YELLOW;
        break;
    case 3:
        color = COLOR_RED;
        break;
    default:
        color = COLOR_RESET;
    }

    std::cout << color << "[" << file << ":" << line << "] " << comment << COLOR_RESET << std::endl;
    // 输出到文件
    std::ofstream logFile("log.txt", std::ios::app); // 以追加模式打开文件
    if (logFile.is_open())
    {
        // 文件中不包含颜色控制字符
        logFile << "[" << file << ":" << line << "] " << comment << std::endl;
        logFile.close();
    }
    else
    {
        std::cerr << "无法打开日志文件 log.txt" << std::endl;
    }
}

