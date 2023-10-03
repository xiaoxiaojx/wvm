#ifndef SRC_PARSER_
#define SRC_PARSER_

#include <string>

#include "module.h"

namespace wvm
{
    template <typename ModuleType>
    class Parser
    {
    private:
        std::string file_name_;

    public:
        Parser(std::string file_name);
        ~Parser();

        std::string file_name();

        std::shared_ptr<ModuleType> parse();
    };
}

#endif // SRC_PARSER_