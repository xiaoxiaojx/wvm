#ifndef SRC_INTERPRETER_H_
#define SRC_INTERPRETER_H_

#include <optional>
#include <array>

#include "opcodes.h"

#define DECLARE_OPCODE_HANDLER_VALID(NAME) \
    static void do##NAME(Executor &, op_handler_info_t = std::nullopt);
#define DECLARE_OPCODE_HANDLER_INVALID(NAME)
#define DECLARE_OPCODE_HANDLER(NAME, OP, VALDITI) \
    DECLARE_OPCODE_HANDLER_##VALDITI(NAME)

namespace wvm
{
    class Executor;

    struct Interpreter
    {
        using op_handler_proto_t = void (*)(Executor &, std::optional<uint32_t>);
        using op_handler_info_t = std::optional<uint32_t>;
        static std::array<op_handler_proto_t, sizeof(uint8_t) * 1 << 8> opTokenHandlers;
        ITERATE_ALL_OPCODE(DECLARE_OPCODE_HANDLER)
    };
}

#endif // SRC_INTERPRETER_H_