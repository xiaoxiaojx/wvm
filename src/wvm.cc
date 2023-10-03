#include <string>
#include "parser.h"
#include "module.h"
#include "decoder.h"

int main(int argc, char *argv[])
{
    std::string file_name = argv[1];

    /** parser module */
    std::shared_ptr<wvm::Parser<wvm::Module, wvm::Decoder>> parser_ptr = std::make_shared<wvm::Parser<wvm::Module, wvm::Decoder>>(file_name);
    std::shared_ptr<wvm::Module> module_ptr = parser_ptr->parse();

    /** instantiate module */
    return 0;
}