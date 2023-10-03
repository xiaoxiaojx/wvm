#ifndef SRC_MODULE_H_
#define SRC_MODULE_H_

#include <string>
#include <fstream>
#include <vector>
#include "decoder.h"

#define MAGIC_NUMBER std::vector<uint8_t>({0x00, 0x61, 0x73, 0x6d})
#define WASM_VERSION std::vector<uint8_t>({0x01, 0x00, 0x00, 0x00})

#define WVM_KEEPALIVE __attribute__((used))

#define SET_STRUCT_DISABLE_COPY_CONSTUCT(TypeName) \
    TypeName(const TypeName &) = delete;           \
    TypeName &operator=(const TypeName &) = delete
#define SET_STRUCT_MOVE_ONLY(TypeName)                   \
    TypeName() = default;                                \
    TypeName(TypeName &&) noexcept = default;            \
    TypeName &operator=(TypeName &&) noexcept = default; \
    SET_STRUCT_DISABLE_COPY_CONSTUCT(TypeName);

namespace wvm
{
    enum class Section : uint8_t
    {
        custom = 0,
        type = 1,
        import = 2,
        func = 3,
        table = 4,
        memory = 5,
        global = 6,
        export_section = 7,
        start = 8,
        element = 9,
        code = 10,
        data = 11
    };

    enum class SectionType : uint8_t
    {
        func = 0x60,
        result = 0x40
    };
    struct TableType
    {
        uint8_t refType;
        uint32_t initial;
        uint32_t maximum;
    };
    struct MemType
    {
        uint32_t initial;
        uint32_t maximum;
    };
    struct GlobalType
    {
        uint8_t valType;
        bool mutability;
    };
    using type_seq_t = std::vector<uint8_t>;
    using external_kind_t = std::variant<uint32_t, TableType, MemType, GlobalType>;
    using func_type_t = std::pair<type_seq_t, type_seq_t>;
    struct FuncDefSec
    {
        SET_STRUCT_MOVE_ONLY(FuncDefSec)
        std::vector<uint8_t> locals;
        std::vector<uint8_t> body;
        FuncDefSec(std::vector<uint8_t> &locals, std::vector<uint8_t> &body)
            : locals(locals), body(body) {}
    };
    struct ImportSec
    {
        SET_STRUCT_MOVE_ONLY(ImportSec)
        std::string modName;
        std::string name;
        uint8_t extKind;
        external_kind_t extMeta;
        ImportSec(std::string modName, std::string name, uint8_t extKind)
            : modName(modName), name(name), extKind(extKind) {}
    };
    struct ExportSec
    {
        SET_STRUCT_MOVE_ONLY(ExportSec)
        std::string name;
        uint8_t extKind;
        uint32_t extIdx;
        ExportSec(std::string name, uint8_t extKind, uint32_t extIdx)
            : name(name), extKind(extKind), extIdx(extIdx) {}
    };
    class Module
    {
    private:
        std::shared_ptr<Decoder> decoder_;

    public:
        Module(std::shared_ptr<Decoder> decoder);
        ~Module();

        std::vector<func_type_t> funcTypes;
        std::vector<ImportSec> imports;
        std::vector<uint32_t> funcTypesIndices;
        std::vector<ExportSec> exports;
        std::vector<FuncDefSec> funcDefs;

        std::shared_ptr<Decoder> decoder();

        void parseMagicNumber();
        void parseVersion();
        void parseSection();
        void parseTypeSection();
        void parseImportSection();
        void parseFunctionSection();
        void parseTableSection();
        void parseMemorySection();
        void parseGlobalSection();
        void parseExportSection();
        void parseCodeSection();

        static bool byteArrayEq(const std::vector<uint8_t> &input1, const std::vector<uint8_t> &input2);
    };
}

#endif // SRC_MODULE_H_