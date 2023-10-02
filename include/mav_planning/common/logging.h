#ifndef MAV_PLANNING_COMMON_LOGGING
#define MAV_PLANNING_COMMON_LOGGING

#include "common.h"


namespace mav_planning
{   
    #define MAX_BUFFER_SIZE 1024

    #define DEBUG(name,fmt,__VA_ARGS__...) logConsole(LogLevel::DEBUG, name, fmt, ## __VA_ARGS__)
    #define INFO(name,fmt,__VA_ARGS__...) logConsole(LogLevel::INFO, name, fmt, ## __VA_ARGS__)
    #define INFO_SPECIAL(name,fmt,__VA_ARGS__...) logConsole(LogLevel::INFO_SPECIAL, name, fmt, ## __VA_ARGS__)
    #define WARN(name,fmt,__VA_ARGS__...) logConsole(LogLevel::WARNING, name, fmt, ## __VA_ARGS__)
    #define ERROR(name,fmt,__VA_ARGS__...) logConsole(LogLevel::ERROR, name, fmt, ## __VA_ARGS__)

    enum PrintColor
    {
        BLACK,
        RED,
        GREEN,
        YELLOW,
        BLUE,
        MAGENTA,
        CYAN,
        WHITE,
        ENDCOLOR
    };

    enum PRINT_MODE { NORMAL, BOLD, ITALIC, UNDERLINED};
    
    enum LogLevel {DEBUG, INFO, INFO_SPECIAL, WARNING, ERROR};

    // void logDebug(std::string name, std::ostream& os);
    void logConsole(LogLevel level, const std::string& name, const char *m, ...);
    // void logInfoSpecial(std::string name, std::ostream& os);
    // void logWarn(std::string name, std::ostream& os);
    // void logError(std::string name, std::ostream& os);
}

#endif /* MAV_PLANNING_COMMON_LOGGING */
