#include <string>
#include "parser.h"

int main(int argc, char *argv[])
{
    std::string file_name = argv[1];

    /** Parser */
    std::shared_ptr<wvm::Parser<wvm::Module>> parser_ptr = std::make_shared<wvm::Parser<wvm::Module>>(file_name);
    std::shared_ptr<wvm::Module> module_ptr = parser_ptr->parse();

    return 0;
}