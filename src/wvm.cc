#include <string>
#include <tuple>

#include "parser.h"
#include "module.h"
#include "decoder.h"
#include "instance.h"
#include "executor.h"

int main(int argc, char *argv[])
{
    std::string file_name = argv[1];

    /** parser module */
    std::shared_ptr<wvm::Parser<wvm::Module, wvm::Decoder>> parser_ptr = std::make_shared<wvm::Parser<wvm::Module, wvm::Decoder>>(file_name);
    std::shared_ptr<wvm::Module> module_ptr = parser_ptr->parse();

    /** instantiate module */
    std::shared_ptr<wvm::Instance> instance_ptr = std::make_shared<wvm::Instance>(module_ptr);
    std::shared_ptr<wvm::Runtime> runtime_ptr = instance_ptr->instantiate();

    /** execute module */
    std::shared_ptr<wvm::Executor> executor_ptr = std::make_shared<wvm::Executor>(runtime_ptr);
    wvm::Executor::engine_result_t ret = executor_ptr->execute(std::optional<uint32_t>{});

    /** print result */
    if (ret.has_value())
    {
        std::cout << std::get<int>(ret.value()) << std::endl;
    }
    return 0;
}