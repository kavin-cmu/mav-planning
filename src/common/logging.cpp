#include "mav_planning/common/logging.h"

namespace mav_planning
{   
    std::ostream& operator<<(std::ostream& os, PrintColor c)
    {
        switch(c)
        {
        case BLACK    : os << "\033[0;30m"; break;
        case RED      : os << "\033[0;31m"; break;
        case GREEN    : os << "\033[0;32m"; break;
        case YELLOW   : os << "\033[0;33m"; break;
        case BLUE     : os << "\033[0;34m"; break;
        case MAGENTA  : os << "\033[0;35m"; break;
        case CYAN     : os << "\033[0;36m"; break;
        case WHITE    : os << "\033[0;37m"; break;
        case ENDCOLOR : os << "\033[0m";    break;
        default       : os << "\033[0;37m";
        }
        return os;
    }

    void logConsole(LogLevel level, const std::string& name, const char *m, ...)
    {
        
        const char *LogLevelString[5] = {"DEB: ", "INF: ", "INF: ", "WRN: ", "ERR: "};
    
        const PrintColor LogLevelColor[5] =  {MAGENTA, WHITE, CYAN, YELLOW, RED};

        va_list __ap;
        va_start(__ap, m);
        char buf[MAX_BUFFER_SIZE];
        vsnprintf(buf, sizeof(buf), m, __ap);
        va_end(__ap);
        buf[MAX_BUFFER_SIZE - 1] = '\0';

        std::cout<<LogLevelColor[level]<<LogLevelString[level]<<"["<<name<<"] "<<buf<<PrintColor::ENDCOLOR<<std::endl;
    }
}