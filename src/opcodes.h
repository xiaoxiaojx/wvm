#ifndef SRC_OPCODES_H_
#define SRC_OPCODES_H_
#include <memory>

#define ITERATE_ALL_OPCODE(V)       \
  V(Unreachable, 0x0, VALID)        \
  V(Nop, 0x1, VALID)                \
  V(Block, 0x2, VALID)              \
  V(Loop, 0x3, VALID)               \
  V(If, 0x4, VALID)                 \
  V(Else, 0x5, VALID)               \
  V(_NA_0x6, 0x6, INVALID)          \
  V(_NA_0x7, 0x7, INVALID)          \
  V(_NA_0x8, 0x8, INVALID)          \
  V(_NA_0x9, 0x9, INVALID)          \
  V(_NA_0xa, 0xa, INVALID)          \
  V(End, 0xb, VALID)                \
  V(Br, 0xc, VALID)                 \
  V(BrIf, 0xd, VALID)               \
  V(BrTable, 0xe, VALID)            \
  V(Return, 0xf, VALID)             \
  V(Call, 0x10, VALID)              \
  V(CallIndirect, 0x11, VALID)      \
  V(_NA_0x12, 0x12, INVALID)        \
  V(_NA_0x13, 0x13, INVALID)        \
  V(_NA_0x14, 0x14, INVALID)        \
  V(_NA_0x15, 0x15, INVALID)        \
  V(_NA_0x16, 0x16, INVALID)        \
  V(_NA_0x17, 0x17, INVALID)        \
  V(_NA_0x18, 0x18, INVALID)        \
  V(_NA_0x19, 0x19, INVALID)        \
  V(Drop, 0x1a, VALID)              \
  V(Select, 0x1b, VALID)            \
  V(_NA_0x1c, 0x1c, INVALID)        \
  V(_NA_0x1d, 0x1d, INVALID)        \
  V(_NA_0x1e, 0x1e, INVALID)        \
  V(_NA_0x1f, 0x1f, INVALID)        \
  V(LocalGet, 0x20, VALID)          \
  V(LocalSet, 0x21, VALID)          \
  V(LocalTee, 0x22, VALID)          \
  V(GlobalGet, 0x23, VALID)         \
  V(GlobalSet, 0x24, VALID)         \
  V(_NA_0x25, 0x25, INVALID)        \
  V(_NA_0x26, 0x26, INVALID)        \
  V(_NA_0x27, 0x27, INVALID)        \
  V(I32LoadMem, 0x28, VALID)        \
  V(I64LoadMem, 0x29, VALID)        \
  V(F32LoadMem, 0x2a, VALID)        \
  V(F64LoadMem, 0x2b, VALID)        \
  V(I32LoadMem8S, 0x2c, VALID)      \
  V(I32LoadMem8U, 0x2d, VALID)      \
  V(I32LoadMem16S, 0x2e, VALID)     \
  V(I32LoadMem16U, 0x2f, VALID)     \
  V(I64LoadMem8S, 0x30, VALID)      \
  V(I64LoadMem8U, 0x31, VALID)      \
  V(I64LoadMem16S, 0x32, VALID)     \
  V(I64LoadMem16U, 0x33, VALID)     \
  V(I64LoadMem32S, 0x34, VALID)     \
  V(I64LoadMem32U, 0x35, VALID)     \
  V(I32StoreMem, 0x36, VALID)       \
  V(I64StoreMem, 0x37, VALID)       \
  V(F32StoreMem, 0x38, VALID)       \
  V(F64StoreMem, 0x39, VALID)       \
  V(I32StoreMem8, 0x3a, VALID)      \
  V(I32StoreMem16, 0x3b, VALID)     \
  V(I64StoreMem8, 0x3c, VALID)      \
  V(I64StoreMem16, 0x3d, VALID)     \
  V(I64StoreMem32, 0x3e, VALID)     \
  V(MemorySize, 0x3f, VALID)        \
  V(MemoryGrow, 0x40, VALID)        \
  V(I32Const, 0x41, VALID)          \
  V(I64Const, 0x42, VALID)          \
  V(F32Const, 0x43, VALID)          \
  V(F64Const, 0x44, VALID)          \
  V(I32Eqz, 0x45, VALID)            \
  V(I32Eq, 0x46, VALID)             \
  V(I32Ne, 0x47, VALID)             \
  V(I32LtS, 0x48, VALID)            \
  V(I32LtU, 0x49, VALID)            \
  V(I32GtS, 0x4a, VALID)            \
  V(I32GtU, 0x4b, VALID)            \
  V(I32LeS, 0x4c, VALID)            \
  V(I32LeU, 0x4d, VALID)            \
  V(I32GeS, 0x4e, VALID)            \
  V(I32GeU, 0x4f, VALID)            \
  V(I64Eqz, 0x50, VALID)            \
  V(I64Eq, 0x51, VALID)             \
  V(I64Ne, 0x52, VALID)             \
  V(I64LtS, 0x53, VALID)            \
  V(I64LtU, 0x54, VALID)            \
  V(I64GtS, 0x55, VALID)            \
  V(I64GtU, 0x56, VALID)            \
  V(I64LeS, 0x57, VALID)            \
  V(I64LeU, 0x58, VALID)            \
  V(I64GeS, 0x59, VALID)            \
  V(I64GeU, 0x5a, VALID)            \
  V(F32Eq, 0x5b, VALID)             \
  V(F32Ne, 0x5c, VALID)             \
  V(F32Lt, 0x5d, VALID)             \
  V(F32Gt, 0x5e, VALID)             \
  V(F32Le, 0x5f, VALID)             \
  V(F32Ge, 0x60, VALID)             \
  V(F64Eq, 0x61, VALID)             \
  V(F64Ne, 0x62, VALID)             \
  V(F64Lt, 0x63, VALID)             \
  V(F64Gt, 0x64, VALID)             \
  V(F64Le, 0x65, VALID)             \
  V(F64Ge, 0x66, VALID)             \
  V(I32Clz, 0x67, VALID)            \
  V(I32Ctz, 0x68, VALID)            \
  V(I32Popcnt, 0x69, VALID)         \
  V(I32Add, 0x6a, VALID)            \
  V(I32Sub, 0x6b, VALID)            \
  V(I32Mul, 0x6c, VALID)            \
  V(I32DivS, 0x6d, VALID)           \
  V(I32DivU, 0x6e, VALID)           \
  V(I32RemS, 0x6f, VALID)           \
  V(I32RemU, 0x70, VALID)           \
  V(I32And, 0x71, VALID)            \
  V(I32Or, 0x72, VALID)             \
  V(I32Xor, 0x73, VALID)            \
  V(I32Shl, 0x74, VALID)            \
  V(I32ShrS, 0x75, VALID)           \
  V(I32ShrU, 0x76, VALID)           \
  V(I32Rotl, 0x77, VALID)           \
  V(I32Rotr, 0x78, VALID)           \
  V(I64Clz, 0x79, VALID)            \
  V(I64Ctz, 0x7a, VALID)            \
  V(I64Popcnt, 0x7b, VALID)         \
  V(I64Add, 0x7c, VALID)            \
  V(I64Sub, 0x7d, VALID)            \
  V(I64Mul, 0x7e, VALID)            \
  V(I64DivS, 0x7f, VALID)           \
  V(I64DivU, 0x80, VALID)           \
  V(I64RemS, 0x81, VALID)           \
  V(I64RemU, 0x82, VALID)           \
  V(I64And, 0x83, VALID)            \
  V(I64Or, 0x84, VALID)             \
  V(I64Xor, 0x85, VALID)            \
  V(I64Shl, 0x86, VALID)            \
  V(I64ShrS, 0x87, VALID)           \
  V(I64ShrU, 0x88, VALID)           \
  V(I64Rotl, 0x89, VALID)           \
  V(I64Rotr, 0x8a, VALID)           \
  V(F32Abs, 0x8b, VALID)            \
  V(F32Neg, 0x8c, VALID)            \
  V(F32Ceil, 0x8d, VALID)           \
  V(F32Floor, 0x8e, VALID)          \
  V(F32Trunc, 0x8f, VALID)          \
  V(F32NearestInt, 0x90, VALID)     \
  V(F32Sqrt, 0x91, VALID)           \
  V(F32Add, 0x92, VALID)            \
  V(F32Sub, 0x93, VALID)            \
  V(F32Mul, 0x94, VALID)            \
  V(F32Div, 0x95, VALID)            \
  V(F32Min, 0x96, VALID)            \
  V(F32Max, 0x97, VALID)            \
  V(F32CopySign, 0x98, VALID)       \
  V(F64Abs, 0x99, VALID)            \
  V(F64Neg, 0x9a, VALID)            \
  V(F64Ceil, 0x9b, VALID)           \
  V(F64Floor, 0x9c, VALID)          \
  V(F64Trunc, 0x9d, VALID)          \
  V(F64NearestInt, 0x9e, VALID)     \
  V(F64Sqrt, 0x9f, VALID)           \
  V(F64Add, 0xa0, VALID)            \
  V(F64Sub, 0xa1, VALID)            \
  V(F64Mul, 0xa2, VALID)            \
  V(F64Div, 0xa3, VALID)            \
  V(F64Min, 0xa4, VALID)            \
  V(F64Max, 0xa5, VALID)            \
  V(F64CopySign, 0xa6, VALID)       \
  V(I32WrapI64, 0xa7, VALID)        \
  V(I32TruncF32S, 0xa8, VALID)      \
  V(I32TruncF32U, 0xa9, VALID)      \
  V(I32TruncF64S, 0xaa, VALID)      \
  V(I32TruncF64U, 0xab, VALID)      \
  V(I64ExtendI32S, 0xac, VALID)     \
  V(I64ExtendI32U, 0xad, VALID)     \
  V(I64TruncF32S, 0xae, VALID)      \
  V(I64TruncF32U, 0xaf, VALID)      \
  V(I64TruncF64S, 0xb0, VALID)      \
  V(I64TruncF64U, 0xb1, VALID)      \
  V(F32SConvertI32, 0xb2, VALID)    \
  V(F32UConvertI32, 0xb3, VALID)    \
  V(F32SConvertI64, 0xb4, VALID)    \
  V(F32UConvertI64, 0xb5, VALID)    \
  V(F32DemoteF64, 0xb6, VALID)      \
  V(F64SConvertI32, 0xb7, VALID)    \
  V(F64UConvertI32, 0xb8, VALID)    \
  V(F64SConvertI64, 0xb9, VALID)    \
  V(F64UConvertI64, 0xba, VALID)    \
  V(F64PromoteF32, 0xbb, VALID)     \
  V(I32ReinterpretF32, 0xbc, VALID) \
  V(I64ReinterpretF64, 0xbd, VALID) \
  V(F32ReinterpretI32, 0xbe, VALID) \
  V(F64ReinterpretI64, 0xbf, VALID)

#define DECLARE_NAMED_ENUM(NAME, OP, _) \
  NAME = OP,

namespace wvm
{

  enum class OpCodes : uint8_t
  {
    ITERATE_ALL_OPCODE(DECLARE_NAMED_ENUM)
  };

}

#endif //   SRC_OPCODES_H_
