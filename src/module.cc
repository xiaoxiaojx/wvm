
#include "module.h"
#include "log.h"

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

    bool Module::byteArrayEq(const std::vector<uint8_t> &input1, const std::vector<uint8_t> &input2)
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
        std::vector<uint8_t> buffer = decoder_->readBytes(4);
        if (!byteArrayEq(MAGIC_NUMBER, buffer))
        {
            LOG("magic header not detected");
            std::exit(1);
        }
    }

    void Module::parseVersion()
    {
        std::vector<uint8_t> buffer = decoder_->readBytes(4);
        if (!byteArrayEq(WASM_VERSION, buffer))
        {
            LOG("unknown binary version");
            std::exit(1);
        }
    }

    void Module::parseSection()
    {
        LOG("parseSection start");
        while (!decoder_->readable().eof())
        {
            Section sectionId = static_cast<Section>(decoder_->readByte());
            std::cout << "sectionId: " << static_cast<int>(sectionId) << std::endl;

            switch (sectionId)
            {
            case Section::type:
                /* Type section -> https://webassembly.github.io/spec/core/binary/modules.html#binary-typesec */
                parseTypeSection();
                break;
            case Section::import:
                /* Import section -> https://webassembly.github.io/spec/core/binary/modules.html#binary-importsec */
                parseImportSection();
                break;
            case Section::func:
                /* Function section -> https://webassembly.github.io/spec/core/binary/modules.html#function-section */
                LOG("parseFunctionSection");
                parseFunctionSection();
                break;
            case Section::table:
                /* Table section -> https://webassembly.github.io/spec/core/binary/modules.html#table-section */
                LOG("parseTableSection");
                parseTableSection();
                break;
            case Section::memory:
                /* Memory section -> https://webassembly.github.io/spec/core/binary/modules.html#memory-section */
                LOG("parseMemorySection");
                parseMemorySection();
                break;
            case Section::global:
                /* Global section -> https://webassembly.github.io/spec/core/binary/modules.html#global-section */
                LOG("parseGlobalSection");
                parseGlobalSection();

            case Section::export_section:
                /* Export section -> https://webassembly.github.io/spec/core/binary/modules.html#export-section */
                LOG("parseExportSection");
                parseExportSection();
                break;

            case Section::code:
                /* Code section -> https://webassembly.github.io/spec/core/binary/modules.html#binary-codesec */
                LOG("parseCodeSection");
                parseCodeSection();
                break;
            default:
                break;
            }
        }
        LOG("parseSection end");
        decoder_->readable().close();
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

    void Module::parseGlobalSection()
    {
    }

    // ; section "Export" (7)
    // 0000016: 07                                        ; section code
    // 0000017: 00                                        ; section size (guess)
    // 0000018: 01                                        ; num exports
    // 0000019: 03                                        ; string length
    // 000001a: 6164 64                                  add  ; export name
    // 000001d: 00                                        ; export kind
    // 000001e: 00                                        ; export func index
    // 0000017: 07                                        ; FIXUP section size
    void Module::parseExportSection()
    {
        WVM_KEEPALIVE const auto sectionSize = decoder_->readU32();
        const auto exportCount = decoder_->readU32();
        for (uint32_t i = 0; i < exportCount; ++i)
        {
            const auto fieldLen = decoder_->readU32();
            const auto fieldStr = decoder_->readStringByBytes(fieldLen);
            const auto extKind = decoder_->readByte();
            const auto extIdx = decoder_->readU32();
            exports.emplace_back(fieldStr, extKind, extIdx);
        }
    }
    // ; section "Code" (10)
    // 000001f: 0a                                        ; section code
    // 0000020: 00                                        ; section size (guess)
    // 0000021: 02                                        ; num functions
    // ; function body 0
    // 0000022: 00                                        ; func body size (guess)
    // 0000023: 00                                        ; local decl count
    // 0000024: 20                                        ; local.get
    // 0000025: 00                                        ; local index
    // 0000026: 20                                        ; local.get
    // 0000027: 01                                        ; local index
    // 0000028: 10                                        ; call
    // 0000029: 01                                        ; function index
    // 000002a: 0b                                        ; end
    // 0000022: 08                                        ; FIXUP func body size
    // ; function body 1
    // 000002b: 00                                        ; func body size (guess)
    // 000002c: 00                                        ; local decl count
    // 000002d: 20                                        ; local.get
    // 000002e: 00                                        ; local index
    // 000002f: 20                                        ; local.get
    // 0000030: 01                                        ; local index
    // 0000031: 41                                        ; i32.const
    // 0000032: 0a                                        ; i32 literal
    // 0000033: 1a                                        ; drop
    // 0000034: 6a                                        ; i32.add
    // 0000035: 0b                                        ; end
    // 000002b: 0a                                        ; FIXUP func body size
    // 0000020: 15                                        ; FIXUP section size
    void Module::parseCodeSection()
    {
        WVM_KEEPALIVE const auto sectionSize = decoder_->readU32();
        const auto funcDefCount = decoder_->readU32();
        for (uint32_t i = 0; i < funcDefCount; ++i)
        {
            const auto bodySize = decoder_->readU32();
            const auto startPos = decoder_->readable().tellg();
            const auto locCount = decoder_->readU32();
            std::vector<uint8_t> locVarTypeVec = {};
            for (uint32_t j = 0; j < locCount; ++j)
            {
                const auto locVarCount = decoder_->readU32();
                const auto locVarType = decoder_->readByte();
                locVarTypeVec.insert(locVarTypeVec.end(), locVarCount, locVarType);
            }
            std::vector<uint8_t> body = decoder_->readBytes(bodySize - (decoder_->readable().tellg() - startPos));
            funcDefs.emplace_back(locVarTypeVec, body);
        }
    }
}