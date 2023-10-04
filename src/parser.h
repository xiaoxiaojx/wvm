#ifndef SRC_PARSER_
#define SRC_PARSER_

#include <string>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace wvm
{
    template <typename ModuleType, typename DecoderType>
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

    template <typename ModuleType, typename DecoderType>
    Parser<ModuleType, DecoderType>::Parser(std::string file_name) : file_name_(file_name)
    {
    }

    template <typename ModuleType, typename DecoderType>
    Parser<ModuleType, DecoderType>::~Parser()
    {
    }

    template <typename ModuleType, typename DecoderType>
    std::string Parser<ModuleType, DecoderType>::file_name()
    {
        return file_name_;
    }

    template <typename ModuleType, typename DecoderType>
    std::shared_ptr<ModuleType> Parser<ModuleType, DecoderType>::parse()
    {
        std::ifstream readable{file_name_, std::ifstream::binary};

        if (!readable.is_open())
        {
            throw std::runtime_error("failed to open file.");
        }

        std::shared_ptr<DecoderType> decoder_ptr = std::make_shared<DecoderType>(readable);

        std::shared_ptr<ModuleType> module_ptr = std::make_shared<ModuleType>(decoder_ptr);
        module_ptr->parseMagicNumber();
        module_ptr->parseVersion();
        module_ptr->parseSection();

        return module_ptr;
    }
}

#endif // SRC_PARSER_