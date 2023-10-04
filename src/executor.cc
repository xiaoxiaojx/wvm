#include <array>
#include <iostream>
#include <algorithm>
#include <type_traits>
#include <cstdlib>
#include <cmath>
#include <cfenv>
#include <limits>

#include "executor.h"

namespace wvm
{
#define DEFAULT_CPU_CORE 4

#define ITERATE_LOAD_MEMOP(V)                  \
    V(I32LoadMem, rt_i32_t, Runtime::rt_i32_t) \
    V(I64LoadMem, rt_i64_t, Runtime::rt_i64_t) \
    V(F32LoadMem, rt_f32_t, Runtime::rt_f32_t) \
    V(F64LoadMem, rt_f64_t, Runtime::rt_f64_t) \
    V(I32LoadMem8S, rt_i32_t, int8_t)          \
    V(I32LoadMem8U, rt_i32_t, uint8_t)         \
    V(I32LoadMem16S, rt_i32_t, int16_t)        \
    V(I32LoadMem16U, rt_i32_t, uint16_t)       \
    V(I64LoadMem8S, rt_i64_t, int8_t)          \
    V(I64LoadMem8U, rt_i64_t, uint8_t)         \
    V(I64LoadMem16S, rt_i64_t, int16_t)        \
    V(I64LoadMem16U, rt_i64_t, uint16_t)       \
    V(I64LoadMem32S, rt_i64_t, int32_t)        \
    V(I64LoadMem32U, rt_i64_t, uint32_t)

#define ITERATE_STORE_MEMOP(V)                  \
    V(I32StoreMem, rt_i32_t, Runtime::rt_i32_t) \
    V(I64StoreMem, rt_i64_t, Runtime::rt_i64_t) \
    V(F32StoreMem, rt_f32_t, Runtime::rt_f32_t) \
    V(F64StoreMem, rt_f64_t, Runtime::rt_f64_t) \
    V(I32StoreMem8, rt_i32_t, uint8_t)          \
    V(I32StoreMem16, rt_i32_t, uint16_t)        \
    V(I64StoreMem8, rt_i64_t, uint8_t)          \
    V(I64StoreMem16, rt_i64_t, uint16_t)        \
    V(I64StoreMem32, rt_i64_t, uint32_t)

#define ITERATE_TRUNCOP(V)                        \
    V(I32TruncF32S, rt_f32_t, rt_i32_t, int32_t)  \
    V(I32TruncF32U, rt_f32_t, rt_i32_t, uint32_t) \
    V(I32TruncF64S, rt_f64_t, rt_i32_t, int32_t)  \
    V(I32TruncF64U, rt_f64_t, rt_i32_t, uint32_t) \
    V(I64TruncF32S, rt_f32_t, rt_i64_t, int64_t)  \
    V(I64TruncF32U, rt_f32_t, rt_i64_t, uint64_t) \
    V(I64TruncF64S, rt_f64_t, rt_i64_t, int64_t)  \
    V(I64TruncF64U, rt_f64_t, rt_i64_t, uint64_t)

#define ITERATE_CONVERTOP(V)                        \
    V(I64ExtendI32S, rt_i32_t, rt_i64_t, SIGNED)    \
    V(I64ExtendI32U, rt_i32_t, rt_i64_t, UNSIGNED)  \
    V(F32SConvertI32, rt_i32_t, rt_f32_t, SIGNED)   \
    V(F32UConvertI32, rt_i32_t, rt_f32_t, UNSIGNED) \
    V(F32SConvertI64, rt_i64_t, rt_f32_t, SIGNED)   \
    V(F32UConvertI64, rt_i64_t, rt_f32_t, UNSIGNED) \
    V(F64SConvertI32, rt_i32_t, rt_f64_t, SIGNED)   \
    V(F64UConvertI32, rt_i32_t, rt_f64_t, UNSIGNED) \
    V(F64SConvertI64, rt_i64_t, rt_f64_t, SIGNED)   \
    V(F64UConvertI64, rt_i64_t, rt_f64_t, UNSIGNED)

#define ITERATE_REINTERPRETOP(V)             \
    V(I32ReinterpretF32, rt_f32_t, rt_i32_t) \
    V(I64ReinterpretF64, rt_f64_t, rt_i64_t) \
    V(F32ReinterpretI32, rt_i32_t, rt_f32_t) \
    V(F64ReinterpretI64, rt_i64_t, rt_f64_t)

#define ITERATE_SIMPLE_BINOP(V)                   \
    V(I32Mul, rt_i32_t, rt_i32_t, rt_i32_t, *)    \
    V(I32Add, rt_i32_t, rt_i32_t, rt_i32_t, +)    \
    V(I32Sub, rt_i32_t, rt_i32_t, rt_i32_t, -)    \
    V(I32And, rt_i32_t, rt_i32_t, rt_i32_t, &)    \
    V(I32Or, rt_i32_t, rt_i32_t, rt_i32_t, |)     \
    V(I32Xor, rt_i32_t, rt_i32_t, rt_i32_t, ^)    \
    V(I32Eq, rt_i32_t, rt_i32_t, rt_i32_t, ==)    \
    V(I32Ne, rt_i32_t, rt_i32_t, rt_i32_t, !=)    \
    V(I32LtU, rt_i32_t, rt_i32_t, imme_u32_t, <)  \
    V(I32LeU, rt_i32_t, rt_i32_t, imme_u32_t, <=) \
    V(I32GtU, rt_i32_t, rt_i32_t, imme_u32_t, >)  \
    V(I32GeU, rt_i32_t, rt_i32_t, imme_u32_t, >=) \
    V(I32LtS, rt_i32_t, rt_i32_t, rt_i32_t, <)    \
    V(I32LeS, rt_i32_t, rt_i32_t, rt_i32_t, <=)   \
    V(I32GtS, rt_i32_t, rt_i32_t, rt_i32_t, >)    \
    V(I32GeS, rt_i32_t, rt_i32_t, rt_i32_t, >=)   \
    V(I64Mul, rt_i64_t, rt_i64_t, rt_i64_t, *)    \
    V(I64Add, rt_i64_t, rt_i64_t, rt_i64_t, +)    \
    V(I64Sub, rt_i64_t, rt_i64_t, rt_i64_t, -)    \
    V(I64And, rt_i64_t, rt_i64_t, rt_i64_t, &)    \
    V(I64Or, rt_i64_t, rt_i64_t, rt_i64_t, |)     \
    V(I64Xor, rt_i64_t, rt_i64_t, rt_i64_t, ^)    \
    V(I64Eq, rt_i64_t, rt_i32_t, rt_i64_t, ==)    \
    V(I64Ne, rt_i64_t, rt_i32_t, rt_i64_t, !=)    \
    V(I64LtU, rt_i64_t, rt_i32_t, imme_u64_t, <)  \
    V(I64LeU, rt_i64_t, rt_i32_t, imme_u64_t, <=) \
    V(I64GtU, rt_i64_t, rt_i32_t, imme_u64_t, >)  \
    V(I64GeU, rt_i64_t, rt_i32_t, imme_u64_t, >=) \
    V(I64LtS, rt_i64_t, rt_i32_t, rt_i64_t, <)    \
    V(I64LeS, rt_i64_t, rt_i32_t, rt_i64_t, <=)   \
    V(I64GtS, rt_i64_t, rt_i32_t, rt_i64_t, >)    \
    V(I64GeS, rt_i64_t, rt_i32_t, rt_i64_t, >=)   \
    V(F32Mul, rt_f32_t, rt_f32_t, rt_f32_t, *)    \
    V(F32Add, rt_f32_t, rt_f32_t, rt_f32_t, +)    \
    V(F32Sub, rt_f32_t, rt_f32_t, rt_f32_t, -)    \
    V(F32Div, rt_f32_t, rt_f32_t, rt_f32_t, /)    \
    V(F32Eq, rt_f32_t, rt_i32_t, rt_f32_t, ==)    \
    V(F32Ne, rt_f32_t, rt_i32_t, rt_f32_t, !=)    \
    V(F32Lt, rt_f32_t, rt_i32_t, rt_f32_t, <)     \
    V(F32Le, rt_f32_t, rt_i32_t, rt_f32_t, <=)    \
    V(F32Gt, rt_f32_t, rt_i32_t, rt_f32_t, >)     \
    V(F32Ge, rt_f32_t, rt_i32_t, rt_f32_t, >=)    \
    V(F64Mul, rt_f64_t, rt_f64_t, rt_f64_t, *)    \
    V(F64Add, rt_f64_t, rt_f64_t, rt_f64_t, +)    \
    V(F64Sub, rt_f64_t, rt_f64_t, rt_f64_t, -)    \
    V(F64Div, rt_f64_t, rt_f64_t, rt_f64_t, /)    \
    V(F64Eq, rt_f64_t, rt_i32_t, rt_f64_t, ==)    \
    V(F64Ne, rt_f64_t, rt_i32_t, rt_f64_t, !=)    \
    V(F64Lt, rt_f64_t, rt_i32_t, rt_f64_t, <)     \
    V(F64Le, rt_f64_t, rt_i32_t, rt_f64_t, <=)    \
    V(F64Gt, rt_f64_t, rt_i32_t, rt_f64_t, >)     \
    V(F64Ge, rt_f64_t, rt_i32_t, rt_f64_t, >=)

#define REF_OPCODE_HANDLER_PTR_VALID(NAME) \
    Interpreter::do##NAME,
#define REF_OPCODE_HANDLER_PTR_INVALID(NAME) \
    nullptr,
#define REF_OPCODE_HANDLER_PTR(NAME, OP, VALIDITY) \
    REF_OPCODE_HANDLER_PTR_##VALIDITY(NAME)
#define CONCAT_PREFIX(X) Runtime::X
#define MAKE_UNSIGNED() std::make_unsigned_t
#define MAKE_SIGNED() std::make_signed_t
#define DECLARE_BASIC_BINOP_METHOD(NAME, VAL_TYPE, RET_TYPE, OP_CAST_TYPE, OP)                                                                                                                               \
    void Interpreter::do##NAME(Executor &executor, op_handler_info_t _)                                                                                                                                      \
    {                                                                                                                                                                                                        \
        executor.opHandlerFTRO<CONCAT_PREFIX(VAL_TYPE), CONCAT_PREFIX(RET_TYPE)>([](auto x, auto y) { return static_cast<CONCAT_PREFIX(OP_CAST_TYPE)>(x) OP static_cast<CONCAT_PREFIX(OP_CAST_TYPE)>(y); }); \
    }
#define DECLARE_MEM_LOAD_OP_METHOD(NAME, T, C)                                              \
    void Interpreter::do##NAME(Executor &executor, op_handler_info_t _)                     \
    {                                                                                       \
        const auto &defaultMem = executor.getEngineData()->rtMems.front();                  \
        const auto [flags, offset] = executor.parseMemImmeInfo();                           \
        const auto ea = executor.retStackValOfRTType<Runtime::rt_i32_t>(false) + offset;    \
        const auto n = sizeof(CONCAT_PREFIX(T));                                            \
        if (ea + n / 8 <= defaultMem.size)                                                  \
        {                                                                                   \
            executor.refFrameFromStack<Runtime::RTValueFrame>().value =                     \
                static_cast<CONCAT_PREFIX(T)>(*reinterpret_cast<C *>(defaultMem.ptr + ea)); \
        }                                                                                   \
        else                                                                                \
        {                                                                                   \
        }                                                                                   \
    }
#define DECLARE_MEM_STORE_OP_METHOD(NAME, T, C)                                     \
    void Interpreter::do##NAME(Executor &executor, op_handler_info_t _)             \
    {                                                                               \
        const auto &defaultMem = executor.getEngineData()->rtMems.front();          \
        const auto [flags, offset] = executor.parseMemImmeInfo();                   \
        const auto c = executor.retStackValOfRTType<CONCAT_PREFIX(T)>();            \
        const auto ea = executor.retStackValOfRTType<Runtime::rt_i32_t>() + offset; \
        const auto n = sizeof(CONCAT_PREFIX(T));                                    \
        if (ea + n / 8 <= defaultMem.size)                                          \
        {                                                                           \
            *reinterpret_cast<C *>(defaultMem.ptr + ea) = static_cast<C>(c);        \
        }                                                                           \
        else                                                                        \
        {                                                                           \
        }                                                                           \
    }
#define DECLARE_TRUNC_OP_METHOD(NAME, PARAM_TYPE, RET_TYPE, CAST)                                   \
    void Interpreter::do##NAME(Executor &executor, op_handler_info_t _)                             \
    {                                                                                               \
        executor.opHandlerFORO<CONCAT_PREFIX(PARAM_TYPE), CONCAT_PREFIX(RET_TYPE)>([](auto v) { \
      v = std::trunc(v); \
      if (!std::isnan(v) && \
          !std::isinf(v) && \
          Util::floatInRange<CAST>(v)) { \
          return static_cast<CAST>(v); \
        } else { \
        } }); \
    }
#define DECLARE_CONVERT_OP_METHOD(NAME, PARAM_TYPE, RET_TYPE, SIGNESS_METHOD)                                                                                          \
    void Interpreter::do##NAME(Executor &executor, op_handler_info_t _)                                                                                                \
    {                                                                                                                                                                  \
        executor.opHandlerFORO<CONCAT_PREFIX(PARAM_TYPE), CONCAT_PREFIX(RET_TYPE)>([](auto v) { return static_cast < MAKE_##SIGNESS_METHOD() < decltype(v) >> (v); }); \
    }
#define DECLARE_REINTERPRET_OP_METHOD(NAME, PARAM_TYPE, RET_TYPE)                                                                                                                                      \
    void Interpreter::do##NAME(Executor &executor, op_handler_info_t _)                                                                                                                                \
    {                                                                                                                                                                                                  \
        executor.opHandlerFORO<CONCAT_PREFIX(PARAM_TYPE), CONCAT_PREFIX(RET_TYPE)>([](auto v) { return *reinterpret_cast<CONCAT_PREFIX(RET_TYPE) *>(static_cast<CONCAT_PREFIX(PARAM_TYPE) *>(&v)); }); \
    }

    Executor::Executor(std::shared_ptr<Runtime> rtIns) : rtIns(rtIns)
    {
    }

    Executor::Executor(uint8_t *pc, shared_module_runtime_t rtIns) : pc(pc), rtIns(rtIns) {}

    Executor::engine_result_t Executor::execute(std::optional<uint32_t> invokeIdx)
    {
        if (!invokeIdx.has_value() && rtIns->rtEntryIdx.has_value())
        {
            invokeIdx = *rtIns->rtEntryIdx;

            std::cout << "invokeIdx: " << invokeIdx.value() << std::endl;
        }
        if (invokeIdx.has_value())
        {
            // [CALL, (IDX), END].
            std::vector<uint8_t> driver = {static_cast<uint8_t>(OpCodes::Call)}; // Driver opcodes.
            const auto bytes = Decoder::encodeVaruint(*invokeIdx);
            driver.insert(driver.end(), bytes.begin(), bytes.end());
            Executor executor(driver.data(), rtIns);
            while (executor.status == EngineStatus::EXECUTING)
            {
                Interpreter::opTokenHandlers[*executor.pc++](executor, std::nullopt);
            }
            // Post-process.
            return executor.postProcess();
        }
        return 0;
    }

    inline Executor::engine_result_t Executor::postProcess()
    {
        // Check return arity.
        const auto &entryFrameOffset = refTrackedTopFrameByType(Runtime::STVariantIndex::ACTIVATION);
        const auto &entryFrame = std::get<Runtime::RTActivFrame>(*entryFrameOffset.ptr);
        const auto &returnArity = entryFrame.returnArity;
        if (returnArity->size() > 0)
        {
            return std::get<Runtime::RTValueFrame>(rtIns->stack.back()).value;
        }
        else
        {
            return std::nullopt;
        }
    }

    std::optional<uint32_t> Executor::getTopFrameIdx(Runtime::STVariantIndex type, uint32_t n)
    {
        const auto &v = frameBitmap.at(static_cast<int>(type));
        return v.size() > n ? std::make_optional(*(v.rbegin() + n)) : std::nullopt;
    }
    Executor::FrameOffset Executor::refTrackedTopFrameByType(Runtime::STVariantIndex type, uint32_t n)
    {
        const auto topIdx = getTopFrameIdx(type, n);
        if (topIdx.has_value())
        {
            return {
                &rtIns->stack.at(*topIdx),
                *topIdx,
            };
        }
        else
        {
        }
    }
    // [entries after End / Else].
    const std::vector<uint8_t *> &Executor::lookupLabelContFromPC()
    { // don't mess this process with interpreter.
        auto &st = store.contStore;
        const auto &iter = st.find(pc);
        if (iter == st.end())
        {
            status = EngineStatus::CRAWLING;
            storedPC = pc;
            size_t pairingCountEnd = 0;
            size_t pairingCountElse = 0;
            std::vector<uint8_t *> conts = {};
            while (status == EngineStatus::CRAWLING)
            {
                const auto op = static_cast<OpCodes>(*pc++);
                switch (op)
                {
                case OpCodes::Block:
                case OpCodes::Loop:
                case OpCodes::If:
                {
                    decodeByteFromPC();
                    pairingCountEnd++;
                    if (op == OpCodes::If)
                    {
                        pairingCountElse++;
                    }
                    break;
                }
                case OpCodes::Br:
                case OpCodes::BrIf:
                {
                    decodeVaruintFromPC<Runtime::relative_depth_t>();
                    break;
                }
                case OpCodes::BrTable:
                {
                    const auto targetCount = decodeVaruintFromPC<Runtime::imme_u32_t>();
                    for (uint32_t i = 0; i < targetCount + 1; ++i)
                    { // Include `default_target`.
                        decodeVaruintFromPC<Runtime::imme_u32_t>();
                    }
                    break;
                }
                case OpCodes::Call:
                {
                    decodeVaruintFromPC<Runtime::index_t>();
                    break;
                }
                case OpCodes::CallIndirect:
                {
                    decodeVaruintFromPC<Runtime::index_t>();
                    decodeByteFromPC();
                    break;
                }
                case OpCodes::LocalGet:
                case OpCodes::LocalSet:
                case OpCodes::LocalTee:
                case OpCodes::GlobalGet:
                case OpCodes::GlobalSet:
                {
                    decodeVaruintFromPC<Runtime::index_t>();
                    break;
                }
                case OpCodes::I32LoadMem:
                case OpCodes::I64LoadMem:
                case OpCodes::F32LoadMem:
                case OpCodes::F64LoadMem:
                case OpCodes::I32LoadMem8S:
                case OpCodes::I32LoadMem8U:
                case OpCodes::I32LoadMem16S:
                case OpCodes::I32LoadMem16U:
                case OpCodes::I64LoadMem8S:
                case OpCodes::I64LoadMem8U:
                case OpCodes::I64LoadMem16S:
                case OpCodes::I64LoadMem16U:
                case OpCodes::I64LoadMem32S:
                case OpCodes::I64LoadMem32U:
                case OpCodes::I32StoreMem:
                case OpCodes::I64StoreMem:
                case OpCodes::F32StoreMem:
                case OpCodes::F64StoreMem:
                case OpCodes::I32StoreMem8:
                case OpCodes::I32StoreMem16:
                case OpCodes::I64StoreMem8:
                case OpCodes::I64StoreMem16:
                case OpCodes::I64StoreMem32:
                {
                    decodeVaruintFromPC<Runtime::index_t>();
                    decodeVaruintFromPC<Runtime::index_t>();
                    break;
                }
                case OpCodes::I32Const:
                {
                    decodeVarintFromPC<Runtime::rt_i32_t>();
                    break;
                }
                case OpCodes::I64Const:
                {
                    decodeVarintFromPC<Runtime::rt_i64_t>();
                    break;
                }
                case OpCodes::F32Const:
                {
                    decodeFloatingPointFromPC<float>();
                    break;
                }
                case OpCodes::F64Const:
                {
                    decodeFloatingPointFromPC<double>();
                    break;
                }
                case OpCodes::Else:
                {
                    if (pairingCountElse == 0)
                    {
                        conts.push_back(pc);
                    }
                    else
                    {
                        pairingCountElse--;
                    }
                    break;
                }
                case OpCodes::End:
                {
                    if (pairingCountEnd == 0)
                    {
                        conts.push_back(pc);
                        status = EngineStatus::EXECUTING;
                    }
                    else
                    {
                        pairingCountEnd--;
                    }
                    break;
                }
                default:
                    break;
                }
            }
            pc = storedPC;
            st[pc] = conts;
            return st[pc];
        }
        else
        {
            return iter->second;
        }
    }

    const void Executor::stopEngine()
    {
        status = EngineStatus::STOPPED;
    }

    struct Util
    {
        template <typename Enumeration>
        static auto asInteger(Enumeration const value)
            -> typename std::underlying_type<Enumeration>::type
        {
            return static_cast<typename std::underlying_type<Enumeration>::type>(value);
        }
        static void printAssistantInfo(bool = true);
        static int getNprocs()
        {
#if defined(LINUX)
            return get_nprocs();
#else
            return DEFAULT_CPU_CORE;
#endif
        }
        template <typename T, typename U>
        static constexpr bool floatInRange(U u)
        {
            if constexpr (std::is_unsigned_v<T>)
            {
                if (u < 0)
                    return false;
            }
            else
            {
                if (u < std::numeric_limits<T>::min() || u > std::numeric_limits<T>::max())
                    return false;
            }
            return true;
        }
        template <typename T>
        static constexpr typename std::enable_if_t<std::is_integral_v<T> && sizeof(T) <= 8, unsigned>
        countPopulation(T value)
        {
            static_assert(sizeof(T) <= 8);
#if __has_builtin(__builtin_popcountll) && __has_builtin(__builtin_popcount)
            return sizeof(T) == 8 ? __builtin_popcountll(static_cast<uint64_t>(value))
                                  : __builtin_popcount(static_cast<uint32_t>(value));
#else
            constexpr uint64_t mask[] = {
                0x5555555555555555,
                0x3333333333333333,
                0x0f0f0f0f0f0f0f0f,
            };
            value = ((value >> 1) &
                     mask[DEFAULT_ELEMENT_INDEX]) +
                    (value & mask[DEFAULT_ELEMENT_INDEX]);
            value = ((value >> 2) &
                     mask[DEFAULT_ELEMENT_INDEX + 1]) +
                    (value & mask[DEFAULT_ELEMENT_INDEX + 1]);
            value = (value >> 4) + value;

            if (sizeof(T) > 1)
        }
        {
            value = ((value >> (sizeof(T) > 1 ? 8 : 0)) &
                     mask[DEFAULT_ELEMENT_INDEX + 2]) +
                    (value & mask[DEFAULT_ELEMENT_INDEX + 2]);
        }
        if (sizeof(T) > 2)
        {
            value = (value >> (sizeof(T) > 2 ? 16 : 0)) + value;
        }
        if (sizeof(T) > 4)
    }
    {
        value = (value >> (sizeof(T) > 4 ? 32 : 0)) + value;
    }
    return static_cast<unsigned>(value & 0xff);
#endif
        }

        template <typename T, unsigned bits = sizeof(T) * 8>
        constexpr static std::enable_if_t<std::is_integral_v<T> && sizeof(T) <= 8, unsigned>
        countTrailingZeros(T value)
        {
            static_assert(bits > 0);
#if __has_builtin(__builtin_ctz) && __has_builtin(__builtin_ctzll)
            return value == 0   ? bits
                   : bits == 64 ? __builtin_ctzll(static_cast<uint64_t>(value))
                                : __builtin_ctz(static_cast<uint32_t>(value));
#else
    using U = std::make_unsigned_t<T>;
    U u = value;
    return CountPopulation(static_cast<U>(~u & (u - 1u)));
#endif
        }

        template <typename T, unsigned bits = sizeof(T) * 8>
        constexpr static std::enable_if_t<std::is_integral_v<T> && sizeof(T) <= 8, unsigned>
        countLeadingZeros(T value)
        {
            static_assert(bits > 0);
#if __has_builtin(__builtin_clzll) && __has_builtin(__builtin_clz)
            return value == 0   ? bits
                   : bits == 64 ? __builtin_clzll(static_cast<uint64_t>(value))
                                : __builtin_clz(static_cast<uint32_t>(value)) - (32 - bits);
#else
    // Binary search algorithm (from "Hacker's Delight").
    if (bits == 1)
    {
        return static_cast<unsigned>(value) ^ 1;
    }
    T upperHalf = value >> (bits / 2);
    T nextValue = upperHalf != 0 ? upperHalf : value;
    unsigned add = upperHalf != 0 ? 0 : bits / 2;
    constexpr unsigned nextBits = bits == 1 ? 1 : bits / 2;
    return countLeadingZeros<T, nextBits>(nextValue) + add;
#endif
        }
    };

    std::array<Interpreter::op_handler_proto_t, sizeof(uint8_t) * 1 << 8> Interpreter::opTokenHandlers = {
        ITERATE_ALL_OPCODE(REF_OPCODE_HANDLER_PTR)};

    void Interpreter::doUnreachable(Executor &executor, op_handler_info_t _)
    {
    }
    void Interpreter::doNop(Executor &executor, op_handler_info_t _) {}
    void Interpreter::doBlock(Executor &executor, op_handler_info_t _)
    {
        const auto returnArityTypes = executor.collectArities();
        const auto cont = executor.lookupLabelContFromPC();
        if (cont.size() > 0)
        {
            executor.pushToStack(Runtime::RTLabelFrame(cont.back(), returnArityTypes));
        }
        else
        {
        }
    }
    void Interpreter::doLoop(Executor &executor, op_handler_info_t _)
    {
        auto *cont = executor.getPC() - 1;
        const auto returnArityTypes = executor.collectArities();
        executor.pushToStack(Runtime::RTLabelFrame(cont, returnArityTypes));
    }
    void Interpreter::doIf(Executor &executor, op_handler_info_t _)
    {
        const auto returnArityTypes = executor.collectArities();
        const auto conts = executor.lookupLabelContFromPC();
        if (conts.size() > 1)
        {
            const auto v = executor.retStackValOfRTType<Runtime::rt_i32_t>();
            executor.pushToStack(Runtime::RTLabelFrame(conts.back(), returnArityTypes));
            if (v == 0)
            {
                executor.setPC(conts.front());
            }
        }
        else
        {
        }
    }
    void Interpreter::doElse(Executor &executor, op_handler_info_t _)
    {
        doBr(executor, 0);
    }
    void Interpreter::doEnd(Executor &executor, op_handler_info_t _)
    {
        doBr(executor, 0); // No forwarding PC.
    }
    void Interpreter::doBr(Executor &executor, op_handler_info_t passedDepth)
    {
        const auto depth = passedDepth.has_value() ? *passedDepth : executor.decodeVaruintFromPC<Runtime::relative_depth_t>();
        const auto labelsCount = executor.getLabelAboveActivFrameCount();
        if (labelsCount > depth)
        {
            // Consume Label frames.
            auto *cont = executor.retFromFrameWithCont<Runtime::RTLabelFrame>(depth);
            if (static_cast<OpCodes>(*(executor.getPC() - 1)) != OpCodes::End)
            {
                executor.setPC(cont);
            }
        }
        else if (labelsCount == depth)
        {
            // Consuem Activ frame.
            doReturn(executor, depth);
        }
        else
        {
        }
    }
    void Interpreter::doBrIf(Executor &executor, op_handler_info_t _)
    {
        const auto v = executor.retStackValOfRTType<Runtime::rt_i32_t>();
        const auto depth = executor.decodeVaruintFromPC<Runtime::relative_depth_t>();
        if (v != 0)
        {
            doBr(executor, depth);
        }
    }
    void Interpreter::doBrTable(Executor &executor, op_handler_info_t _)
    {
        const auto entries = executor.parseBrTableInfo();
        const auto v = executor.retStackValOfRTType<Runtime::rt_i32_t>();
        if (v < entries.size())
        {
            doBr(executor, entries.at(v));
        }
        else
        {
            doBr(executor, entries.back());
        }
    }
    void Interpreter::doReturn(Executor &executor, op_handler_info_t labelDepth)
    {
        const auto activIdx = executor.getTopFrameIdx(Runtime::STVariantIndex::ACTIVATION);
        if (activIdx.has_value() && *activIdx == 0)
        {
            executor.stopEngine();
        }
        else
        {
            const auto depth = labelDepth.value_or(executor.getLabelAboveActivFrameCount());
            executor.setPC(
                executor.retFromFrameWithCont<Runtime::RTActivFrame>(depth));
        }
    }

    void Interpreter::doCall(Executor &executor, op_handler_info_t passedFuncIdx)
    {
        const auto idx = passedFuncIdx.has_value() ? *passedFuncIdx : executor.decodeVaruintFromPC<Runtime::index_t>();
        const auto &descriptor = executor.getEngineData()->rtFuncDescriptor.at(idx);
        auto paramCount = descriptor.funcType->first.size();
        auto rtLocals = descriptor.localsDefault; // Copied.
        std::cout << "doCall rtLocals.size: " << rtLocals.size() << std::endl;

        // Set up func parameters.
        if (paramCount > 0)
        {
            for (auto i = 0; i < paramCount; ++i)
            {
                const auto &vf = executor.refFrameFromStack<Runtime::RTValueFrame>();
                if (vf.value.index() == rtLocals.at(i).index())
                {
                    rtLocals.at(i) = vf.value;
                    executor.popFromStack();
                }
                else
                {
                }
            }
        }
        // Construct frame (locals + artiy).
        executor.pushToStack(
            Runtime::RTActivFrame(
                rtLocals,
                executor.getPC(),
                &descriptor.funcType->second));
        // Redirection.
        executor.setPC(descriptor.codeEntry);
        std::cout << "doCall executor.setPC: " << descriptor.codeEntry << std::endl;
    }
    void Interpreter::doCallIndirect(Executor &executor, op_handler_info_t _)
    {
        const auto &engineData = executor.getEngineData();
        const auto &defaultTable = engineData->rtTables.front(); // Restricted to only 1 table in MVP.
        const auto sigIdx = executor.decodeVaruintFromPC<Runtime::index_t>();
        [[maybe_unused]] const auto tblIdx = executor.decodeByteFromPC(); // reserved.
        const auto &funcTypesRef = engineData->module->funcTypes;
        if (funcTypesRef.size() > sigIdx)
        {
            const auto &expectedType = funcTypesRef.at(sigIdx);
            const auto funcIdx = executor.retStackValOfRTType<Runtime::rt_i32_t>();
            if (defaultTable.size() > funcIdx)
            {
                executor.validateTypeWithFuncIdx(expectedType, funcIdx); // May throw.
                doCall(executor, funcIdx);
            }
            else
            {
            }
        }
        else
        {
        }
    }
    void Interpreter::doDrop(Executor &executor, op_handler_info_t _)
    {
        executor.refFrameFromStack<Runtime::RTValueFrame>();
        executor.popFromStack();
    }
    void Interpreter::doSelect(Executor &executor, op_handler_info_t _)
    {
        const auto v = executor.retStackValOfRTType<Runtime::rt_i32_t>();
        const auto &vy = executor.refFrameFromStack<Runtime::RTValueFrame>(); // Top.
        const auto &vx = executor.refFrameFromStack<Runtime::RTValueFrame>(1);
        if (vy.value.index() == vx.value.index())
        {
            executor.eraseFrameFromStack(v == 0 ? 1 : 0);
        }
        else
        {
        }
    }
    void Interpreter::doLocalGet(Executor &executor, op_handler_info_t _)
    {
        const auto idx = executor.decodeVaruintFromPC<Runtime::index_t>();
        const auto &frameOffset = executor.refTrackedTopFrameByType(Runtime::STVariantIndex::ACTIVATION);
        const auto &locals = std::get<Runtime::RTActivFrame>(*frameOffset.ptr).locals;
        if (locals.size() >= idx + 1)
        {
            executor.pushToStack(locals.at(idx));
        }
        else
        {
        }
    }
    void Interpreter::doLocalSet(Executor &executor, op_handler_info_t fromTee)
    {
        const auto localIdx = executor.decodeVaruintFromPC<Runtime::index_t>();
        const auto &activFrameOffset = executor.refTrackedTopFrameByType(Runtime::STVariantIndex::ACTIVATION);
        const auto &valueFrame = executor.refFrameFromStack<Runtime::RTValueFrame>();
        auto &locals = std::get<Runtime::RTActivFrame>(*activFrameOffset.ptr).locals;
        if (locals.at(localIdx).index() == valueFrame.value.index())
        {
            locals.at(localIdx) = valueFrame.value;
        }
        else
        {
        }
        if (!fromTee.has_value())
        {
            executor.popFromStack();
        }
    }
    void Interpreter::doLocalTee(Executor &executor, op_handler_info_t _)
    {
        doLocalSet(executor, OPTIONAL_SYM_BOOL_TRUE);
    }
    void Interpreter::doGlobalGet(Executor &executor, op_handler_info_t _)
    {
        const auto idx = executor.decodeVaruintFromPC<Runtime::index_t>();
        auto &rtGlobals = executor.getEngineData()->rtGlobals;
        if (rtGlobals.size() > idx)
        {
            executor.pushToStack(Runtime::RTValueFrame(rtGlobals.at(idx)));
        }
        else
        {
        }
    }
    void Interpreter::doGlobalSet(Executor &executor, op_handler_info_t _)
    {
        const auto idx = executor.decodeVaruintFromPC<Runtime::index_t>();
        const auto &engineData = executor.getEngineData();
        auto &rtGlobals = engineData->rtGlobals;
        if (rtGlobals.size() > idx)
        {
            const auto mutability = engineData->module->globals.at(idx).globalType.mutability;
            const auto &v = executor.refFrameFromStack<Runtime::RTValueFrame>();
            if (mutability && v.value.index() == rtGlobals.at(idx).index())
            {
                rtGlobals.at(idx) = v.value;
                executor.popFromStack();
            }
            else
            {
            }
        }
        else
        {
        }
    }
    void Interpreter::doI32Const(Executor &executor, op_handler_info_t _)
    {
        executor.pushToStack(
            Runtime::RTValueFrame(executor.decodeVarintFromPC<Runtime::rt_i32_t, Runtime::runtime_value_t>()));
    }
    void Interpreter::doI64Const(Executor &executor, op_handler_info_t _)
    {
        executor.pushToStack(
            Runtime::RTValueFrame(executor.decodeVarintFromPC<Runtime::rt_i64_t, Runtime::runtime_value_t>()));
    }
    void Interpreter::doF32Const(Executor &executor, op_handler_info_t _)
    {
        executor.pushToStack(Runtime::RTValueFrame(executor.decodeFloatingPointFromPC<Runtime::rt_f32_t>()));
    }
    void Interpreter::doF64Const(Executor &executor, op_handler_info_t _)
    {
        executor.pushToStack(Runtime::RTValueFrame(executor.decodeFloatingPointFromPC<Runtime::rt_f64_t>()));
    }
    void Interpreter::doMemorySize(Executor &executor, op_handler_info_t _)
    {
        const auto &rtMems = executor.getEngineData()->rtMems;
        [[maybe_unused]] const auto reserved = executor.decodeByteFromPC();
        if (rtMems.size() > 0)
        {
            const auto &defaultMem = rtMems.front();
            executor.pushToStack(
                Runtime::RTValueFrame(
                    static_cast<Runtime::rt_i32_t>(defaultMem.size)));
        }
        else
        {
        }
    }
    void Interpreter::doMemoryGrow(Executor &executor, op_handler_info_t _)
    {
        const auto &rtMems = executor.getEngineData()->rtMems;
        [[maybe_unused]] const auto reserved = executor.decodeByteFromPC();
        if (rtMems.size() > 0)
        {
            const auto &defaultMem = rtMems.front();
            const auto sz = defaultMem.size / WASM_PAGE_SIZE_IN_BYTE;
            const auto n = executor.retStackValOfRTType<Runtime::rt_i32_t>();
            const auto size = n + sz;
            executor.pushToStack(
                Runtime::RTValueFrame(static_cast<Runtime::rt_i32_t>(executor.resizeMem(size))));
        }
        else
        {
        }
    }
    void Interpreter::doI32Eqz(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto v)
                                                                     { return v == 0; });
    }
    void Interpreter::doI64Eqz(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_i64_t, Runtime::rt_i32_t>([](auto v)
                                                                     { return v == 0; });
    }
    void Interpreter::doI32Clz(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto v)
                                                                     { return Util::countLeadingZeros(v); });
    }
    void Interpreter::doI32Ctz(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto v)
                                                                     { return Util::countTrailingZeros(v); });
    }
    void Interpreter::doI32Popcnt(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto v)
                                                                     { return Util::countPopulation(v); });
    }
    void Interpreter::doI32DivS(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto x, auto y)
                                                                     {
    const auto sx = static_cast<std::make_signed_t<decltype(x)>>(x);
    const auto sy = static_cast<std::make_signed_t<decltype(y)>>(y);
    if (sy != 0) {
      if (!(sy == -1 && sx == std::numeric_limits<int32_t>::min())) {
        return sx / sy;
      } else {
      }
    } else {
    } });
    }
    void Interpreter::doI32DivU(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    const auto uy = static_cast<std::make_unsigned_t<decltype(y)>>(y);
    if (uy != 0) {
      return ux / uy;
    } else {
    } });
    }
    void Interpreter::doI32RemS(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto x, auto y)
                                                                     {
    const auto sx = static_cast<std::make_signed_t<decltype(x)>>(x);
    const auto sy = static_cast<std::make_signed_t<decltype(y)>>(y);
    if (sy != 0) {
      return sx % sy;
    } else {
    } });
    }
    void Interpreter::doI32RemU(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    const auto uy = static_cast<std::make_unsigned_t<decltype(y)>>(y);
    if (uy != 0) {
      return ux % uy;
    } else {
    } });
    }
    void Interpreter::doI32Shl(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto x, auto y)
                                                                     { return x << (y & 0x1f); });
    }
    void Interpreter::doI32ShrS(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto x, auto y)
                                                                     {
    const auto sx = static_cast<std::make_signed_t<decltype(x)>>(x);
    const auto sy = static_cast<std::make_signed_t<decltype(y)>>(y);
    return sx >> (sy & 0x1f); });
    }
    void Interpreter::doI32ShrU(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    const auto uy = static_cast<std::make_unsigned_t<decltype(y)>>(y);
    return ux >> (uy & 0x1f); });
    }
    void Interpreter::doI32Rotl(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    return (ux << (y & 0x1f)) | (ux >> (32 - (y & 0x1f))); });
    }
    void Interpreter::doI32Rotr(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i32_t, Runtime::rt_i32_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    return (ux >> (y & 0x1f)) | (ux << ((32 - y) & 0x1f)); });
    }
    void Interpreter::doI64Clz(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto v)
                                                                     { return Util::countLeadingZeros(v); });
    }
    void Interpreter::doI64Ctz(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto v)
                                                                     { return Util::countTrailingZeros(v); });
    }
    void Interpreter::doI64Popcnt(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto v)
                                                                     { return Util::countPopulation(v); });
    }
    void Interpreter::doI64DivS(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto x, auto y)
                                                                     {
    const auto sx = static_cast<std::make_signed_t<decltype(x)>>(x);
    const auto sy = static_cast<std::make_signed_t<decltype(y)>>(y);
    if (sy != 0) {
      if (!(sy == -1 && sx == std::numeric_limits<int64_t>::min())) {
        return sx / sy;
      } else {
      }
    } else {
    } });
    }
    void Interpreter::doI64DivU(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    const auto uy = static_cast<std::make_unsigned_t<decltype(y)>>(y);
    if (uy != 0) {
      return ux / uy;
    } else {
    } });
    }
    void Interpreter::doI64RemS(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto x, auto y)
                                                                     {
    const auto sx = static_cast<std::make_signed_t<decltype(x)>>(x);
    const auto sy = static_cast<std::make_signed_t<decltype(y)>>(y);
    if (sy != 0) {
      return sx % sy;
    } else {
    } });
    }
    void Interpreter::doI64RemU(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    const auto uy = static_cast<std::make_unsigned_t<decltype(y)>>(y);
    if (uy != 0) {
      return ux % uy;
    } else {
    } });
    }
    void Interpreter::doI64Shl(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto x, auto y)
                                                                     { return x << (y & 0x3f); });
    }
    void Interpreter::doI64ShrS(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto x, auto y)
                                                                     {
    const auto sx = static_cast<std::make_signed_t<decltype(x)>>(x);
    const auto sy = static_cast<std::make_signed_t<decltype(y)>>(y);
    return sx >> (sy & 0x3f); });
    }
    void Interpreter::doI64ShrU(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    const auto uy = static_cast<std::make_unsigned_t<decltype(y)>>(y);
    return ux >> (uy & 0x3f); });
    }
    void Interpreter::doI64Rotl(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    return (ux << (y & 0x3f)) | (ux >> (64 - (y & 0x3f))); });
    }
    void Interpreter::doI64Rotr(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_i64_t, Runtime::rt_i64_t>([](auto x, auto y)
                                                                     {
    const auto ux = static_cast<std::make_unsigned_t<decltype(x)>>(x);
    return (ux >> (y & 0x3f)) | (ux << ((64 - y) & 0x3f)); });
    }
    void Interpreter::doF32Abs(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto v)
                                                                     { return std::abs(v); });
    }
    void Interpreter::doF32Neg(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto v)
                                                                     { return -v; });
    }
    void Interpreter::doF32Ceil(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto v)
                                                                     { return std::ceil(v); });
    }
    void Interpreter::doF32Floor(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto v)
                                                                     { return std::floor(v); });
    }
    void Interpreter::doF32Trunc(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto v)
                                                                     { return std::trunc(v); });
    }
    void Interpreter::doF32NearestInt(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto v)
                                                                     {
    std::fesetround(FE_TONEAREST);
    return std::nearbyint(v); });
    }
    void Interpreter::doF32Sqrt(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto v)
                                                                     { return std::sqrt(v); });
    }
    void Interpreter::doF32Min(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto x, auto y)
                                                                     { return std::min(x, y); });
    }
    void Interpreter::doF32Max(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto x, auto y)
                                                                     { return std::max(x, y); });
    }
    void Interpreter::doF32CopySign(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_f32_t, Runtime::rt_f32_t>([](auto x, auto y)
                                                                     { return std::copysign(x, y); });
    }
    void Interpreter::doF64Abs(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto v)
                                                                     { return std::abs(v); });
    }
    void Interpreter::doF64Neg(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto v)
                                                                     { return -v; });
    }
    void Interpreter::doF64Ceil(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto v)
                                                                     { return std::ceil(v); });
    }
    void Interpreter::doF64Floor(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto v)
                                                                     { return std::floor(v); });
    }
    void Interpreter::doF64Trunc(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto v)
                                                                     { return std::trunc(v); });
    }
    void Interpreter::doF64NearestInt(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto v)
                                                                     {
    std::fesetround(FE_TONEAREST);
    return std::nearbyint(v); });
    }
    void Interpreter::doF64Sqrt(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto v)
                                                                     { return std::sqrt(v); });
    }
    void Interpreter::doF64Min(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto x, auto y)
                                                                     { return std::min(x, y); });
    }
    void Interpreter::doF64Max(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto x, auto y)
                                                                     { return std::max(x, y); });
    }
    void Interpreter::doF64CopySign(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFTRO<Runtime::rt_f64_t, Runtime::rt_f64_t>([](auto x, auto y)
                                                                     { return std::copysign(x, y); });
    }
    void Interpreter::doI32WrapI64(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_i64_t, Runtime::rt_i32_t>([](auto v)
                                                                     { return v & 0xffffffff; });
    }
    void Interpreter::doF32DemoteF64(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f64_t, Runtime::rt_f32_t>([](auto v)
                                                                     { return v; });
    }
    void Interpreter::doF64PromoteF32(Executor &executor, op_handler_info_t _)
    {
        executor.opHandlerFORO<Runtime::rt_f32_t, Runtime::rt_f64_t>([](auto v)
                                                                     { return v; });
    }

    ITERATE_SIMPLE_BINOP(DECLARE_BASIC_BINOP_METHOD)
    ITERATE_LOAD_MEMOP(DECLARE_MEM_LOAD_OP_METHOD)
    ITERATE_STORE_MEMOP(DECLARE_MEM_STORE_OP_METHOD)
    ITERATE_TRUNCOP(DECLARE_TRUNC_OP_METHOD)
    ITERATE_CONVERTOP(DECLARE_CONVERT_OP_METHOD)
    ITERATE_REINTERPRETOP(DECLARE_REINTERPRET_OP_METHOD)
}