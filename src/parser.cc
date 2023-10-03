#include "parser.h"
#include <fstream>
#include <iostream>

namespace wvm
{
    template <typename ModuleType>
    Parser<ModuleType>::Parser(std::string file_name) : file_name_(file_name)
    {
    }

    template <typename ModuleType>
    Parser<ModuleType>::~Parser()
    {
    }

    template <typename ModuleType>
    std::string Parser<ModuleType>::file_name()
    {
        return file_name_;
    }

    template <typename ModuleType>
    std::shared_ptr<ModuleType> Parser<ModuleType>::parse()
    {
        std::ifstream readable{file_name_, std::ifstream::binary};

        if (!readable.is_open())
        {
            std::cerr << "failed to open file." << std::endl;
            return NULL;
        }

        std::shared_ptr<wvm::Decoder> decoder_ptr = std::make_shared<wvm::Decoder>(readable);

        std::shared_ptr<ModuleType> module_ptr = std::make_shared<ModuleType>(decoder_ptr);
        module_ptr->parseMagicNumber();
        module_ptr->parseVersion();
        module_ptr->parseSection();

        return module_ptr;
    }
}