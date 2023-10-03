
#include "module.h"
#include "log.h"
#include "constants.h"

namespace wvm
{
    Module::Module(std::shared_ptr<Decoder> decoder) : decoder_(decoder)
    {
    }

    Module::~Module()
    {
    }

    std::shared_ptr<Decoder> Module::decoder()
    {
        return decoder_;
    }

    bool Module::byteArrayEq(const std::vector<char> &input1, const std::vector<char> &input2)
    {
        if (input1.size() != input2.size())
        {
            return false;
        }

        for (std::size_t i = 0; i < input1.size(); ++i)
        {
            if (input1[i] != input2[i])
            {
                return false;
            }
        }

        return true;
    }

    void Module::parseMagicNumber()
    {
        std::vector<char> buffer = decoder_->readBytes(4);
        LOG("parseMagicNumber -> ", buffer);

        if (!byteArrayEq(MAGIC_NUMBER, buffer))
        {
            LOG("magic header not detected");
            std::exit(1);
        }
    }

    void Module::parseVersion()
    {
        std::vector<char> buffer = decoder_->readBytes(4);
        LOG("parseVersion -> ", buffer);

        if (!byteArrayEq(WASM_VERSION, buffer))
        {
            LOG("unknown binary version");
            std::exit(1);
        }
    }

    void Module::parseSection()
    {
        Section sectionId = static_cast<Section>(decoder_->readByte());

        switch (sectionId)
        {
        case Section::type:
            /* code */
            LOG("parseTypeSection");
            parseTypeSection();
            break;
        case Section::import:
            /* code */
            LOG("parseImportSection");
            parseImportSection();
            break;
        case Section::func:
            /* code */
            LOG("parseFunctionSection");
            parseFunctionSection();
            break;
        default:
            break;
        }
    }
    // ; func type 0
    // 000000b: 60                                        ; func
    // 000000c: 02                                        ; num params
    // 000000d: 7f                                        ; i32
    // 000000e: 7f                                        ; i32
    // 000000f: 01                                        ; num results
    // 0000010: 7f                                        ; i32
    // 0000009: 07                                        ; FIXUP section size
    void Module::parseTypeSection()
    {
        WVM_KEEPALIVE const auto sectionSize = decoder_->readU32();
        const auto numberOfTypes = decoder_->readU32();
        for (uint32_t i = 0; i < numberOfTypes; ++i)
        {
            SectionType sectionType = static_cast<SectionType>(decoder_->readByte());
            if (sectionType == SectionType::func)
            {
                auto pair = std::make_pair(
                    std::vector<uint8_t>{},
                    std::vector<uint8_t>{});
                const auto paramsCount = decoder_->readU32();
                for (uint32_t i = 0; i < paramsCount; ++i)
                {
                    pair.first.push_back(decoder_->readByte());
                }
                const auto resultCount = decoder_->readU32();
                for (uint32_t i = 0; i < resultCount; ++i)
                {
                    pair.second.push_back(decoder_->readByte());
                }
                funcTypes.emplace_back(std::move(pair));
            }
            else
            {
            }
        }
    }

    void Module::parseImportSection()
    {
    }

    // ; section "Function" (3)
    // 0000011: 03                                        ; section code
    // 0000012: 00                                        ; section size (guess)
    // 0000013: 02                                        ; num functions
    // 0000014: 00                                        ; function 0 signature index
    // 0000015: 00                                        ; function 1 signature index
    // 0000012: 03                                        ; FIXUP section size
    void Module::parseFunctionSection()
    {
        WVM_KEEPALIVE const auto sectionSize = decoder_->readU32();
        const auto funcIndexCount = decoder_->readU32();
        for (uint32_t i = 0; i < funcIndexCount; ++i)
        {
            funcTypesIndices.push_back(decoder_->readU32());
        }
    }

    void Module::parseTableSection()
    {
    }

    void Module::parseMemorySection()
    {
    }
}