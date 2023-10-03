#ifndef SRC_EXECUTOR_H_
#define SRC_EXECUTOR_H_

#include <optional>
#include <unordered_map>

#include "instance.h"
#include "opcodes.h"
#include "module.h"

constexpr uint8_t MAGIC_BYTES_COUNT = 4;
constexpr uint8_t VER_BYTES_COUNT = 4;
constexpr uint32_t VALID_MAGIC = 0x6D736100;
constexpr uint8_t VALID_VERSION = 0x1;
constexpr size_t WASM_PAGE_SIZE_IN_BYTE = 64 * 1024;
constexpr size_t WASM_MAX_PAGES = 1 << 16;
constexpr uint8_t EXT_KIND_TAB = 0x1;
constexpr uint8_t EXT_KIND_MEM = 0x2;
constexpr uint8_t EXT_KIND_GLB = 0x3;
constexpr uint32_t OPTIONAL_SYM_BOOL_TRUE = 1;
constexpr uint32_t OPTIONAL_SYM_BOOL_FALSE = 0;

/* Magic Num */
constexpr uint8_t MAGIC_CON_OPCODE_PLUS_TYPE = 0xc0;
constexpr uint8_t MAGIC_VAR_INDEX_PLUS_TYPE = 0x7f;

/* Others */
constexpr auto SCREEN_ARG_SETW_OFFSET = 23;
constexpr auto INPUT_ENTRY_KEY_NAME = "entryFuncName";
constexpr auto INPUT_ENTRY_KEY_ARG = "entryArgs";

#define DECLARE_OPCODE_HANDLER_VALID(NAME) \
    static void do##NAME(Executor &, op_handler_info_t = std::nullopt);
#define DECLARE_OPCODE_HANDLER_INVALID(NAME)
#define DECLARE_OPCODE_HANDLER(NAME, OP, VALDITI) \
    DECLARE_OPCODE_HANDLER_##VALDITI(NAME)

namespace wvm
{
    class Executor
    {
        using engine_result_t = const std::optional<Runtime::runtime_value_t>;
        enum class EngineStatus : uint8_t
        {
            EXECUTING,
            CRAWLING, // Crawling continuation (for labels).
            STOPPED,
        };
        struct FrameOffset
        {
            Runtime::stack_frame_t *ptr;
            uint32_t offset;
        };
        struct MemImme
        {
            uint32_t flags;
            uint32_t offset;
        };
        uint8_t *pc;
        uint8_t *storedPC;
        shared_module_runtime_t rtIns;
        EngineStatus status = EngineStatus::EXECUTING;
        std::optional<Runtime::relative_depth_t> brIfDepth;
        std::vector<std::vector<uint32_t>> frameBitmap = {{}, {}, {}};
        size_t labelAboveActivFrameCount = 0;
        struct
        {
            std::unordered_map<uint8_t *, std::tuple<Runtime::runtime_value_t, uint8_t *>> rtValStore;
            // Since we only use uint32_t for now, here we get rid of variant.
            std::unordered_map<uint8_t *, std::tuple<Runtime::imme_u32_t, uint8_t *>> immeValStore;
            std::unordered_map<uint8_t *, std::vector<uint8_t *>> contStore;
        } store;

    public:
        Executor(std::shared_ptr<Runtime>);
        Executor(uint8_t *pc, shared_module_runtime_t rtIns);

        const auto getCurrentStatus() const { return status; }
        const void stopEngine();
        auto getEngineData() { return rtIns; }
        std::optional<uint32_t> getTopFrameIdx(Runtime::STVariantIndex, uint32_t = 0);
        void setInFrameBitmap(Runtime::STVariantIndex type, uint32_t idx)
        {
            frameBitmap.at(static_cast<int>(type)).push_back(idx);
        }
        void eraseFromFrameBitmap(Runtime::STVariantIndex type, uint32_t n)
        {
            auto &v = frameBitmap.at(static_cast<int>(type));
            v.erase(v.end() - n, v.end());
        }
        Executor::FrameOffset refTrackedTopFrameByType(Runtime::STVariantIndex, uint32_t = 0);
        auto getLabelAboveActivFrameCount() { return labelAboveActivFrameCount; }
        // PC-related methods.
        auto getPC() { return pc; }
        void setPC(uint8_t *addr) { pc = addr; }
        auto movPC(size_t steps = 1)
        {
            pc += steps;
            return pc;
        }
        const std::vector<uint8_t *> &lookupLabelContFromPC();
        auto decodeByteFromPC()
        {
            return *reinterpret_cast<uint8_t *>(pc++);
        }
        template <typename T>
        decltype(auto) decodeVaruintFromPC()
        {
            const auto entryPC = pc;
            auto &st = store.immeValStore;
            const auto &iter = st.find(entryPC);
            if (iter == st.end())
            {
                const auto v = Decoder::decodeVaruint<T>(pc);
                st[entryPC] = std::make_tuple(v, pc);
                const auto &[rtVal, newPC] = st[entryPC];
                return rtVal;
            }
            else
            {
                const auto &[rtVal, newPC] = iter->second;
                pc = newPC;
                return rtVal;
            }
        }
        template <typename T, typename U = T>
        decltype(auto) decodeVarintFromPC()
        {
            const auto entryPC = pc;
            auto &st = store.rtValStore;
            const auto &iter = st.find(entryPC);
            if (iter == st.end())
            {
                const auto v = Decoder::decodeVarint<T>(pc);
                st[entryPC] = std::make_tuple(v, pc);
                const auto &[rtVal, newPC] = st[entryPC];
                if constexpr (std::is_same_v<U, Runtime::runtime_value_t>)
                {
                    return rtVal;
                }
                else
                {
                    return std::get<T>(rtVal);
                }
            }
            else
            {
                const auto &[rtVal, newPC] = iter->second;
                pc = newPC;
                if constexpr (std::is_same_v<U, Runtime::runtime_value_t>)
                {
                    return rtVal;
                }
                else
                {
                    return std::get<T>(rtVal);
                }
            }
        }
        template <typename T>
        T decodeFloatingPointFromPC()
        {
            static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>, "Invalid de-referenced type.");
            const auto fv = *reinterpret_cast<T *>(pc);
            movPC(sizeof(T));
            return fv;
        }
        // Stack-related methods.
        template <typename T>
        auto &refFrameFromStack(size_t pos = 0)
        {
            try
            {

                std::cout << "rtIns->stack.size: " << rtIns->stack.size() << std::endl;
                return std::get<T>(
                    rtIns->stack.at(rtIns->stack.size() - 1 - pos));
            }
            catch (...)
            {
                // Exception::terminate(Exception::ErrorType::STACK_VAL_TYPE_MISMATCH);
            }
        }
        template <typename T>
        void pushToStack(T &&frame)
        {
            rtIns->stack.emplace_back(std::forward<T>(frame));
            const auto updateIdx = rtIns->stack.size() - 1;
            if constexpr (std::is_same_v<T, Runtime::RTActivFrame>)
            {
                labelAboveActivFrameCount = 0;
                setInFrameBitmap(Runtime::STVariantIndex::ACTIVATION, updateIdx);
            }
            if constexpr (std::is_same_v<T, Runtime::RTLabelFrame>)
            {
                labelAboveActivFrameCount++;
                setInFrameBitmap(Runtime::STVariantIndex::LABEL, updateIdx);
            }
        }
        void eraseRangeFromStack(uint32_t startIdx, uint32_t posToTop)
        {
            rtIns->stack.erase(rtIns->stack.begin() + startIdx, rtIns->stack.end() - posToTop);
        }
        void eraseFrameFromStack(uint32_t IdxFromTop)
        {
            rtIns->stack.erase(rtIns->stack.end() - IdxFromTop - 1, rtIns->stack.end() - IdxFromTop);
        }
        void popFromStack() { rtIns->stack.pop_back(); }
        template <typename T>
        auto retStackValOfRTType(bool pop = true)
        {
            try
            {
                const T v = std::get<T>(
                    std::get<Runtime::RTValueFrame>(rtIns->stack.back()).value);
                if (pop)
                    rtIns->stack.pop_back();
                return v;
            }
            catch (...)
            {
                // Exception::terminate(Exception::ErrorType::STACK_VAL_TYPE_MISMATCH);
            }
        }
        void validateArity(const type_seq_t &arity)
        {
            if (arity.size() > 0)
            {
                for (auto i = 0; i < arity.size(); ++i)
                {
                    try
                    {
                        const auto &frame = std::get<Runtime::RTValueFrame>(*(rtIns->stack.rbegin() + i));
                        if ((frame.value.index() + arity.at(i)) != MAGIC_VAR_INDEX_PLUS_TYPE)
                        {
                            // Exception::terminate(Exception::ErrorType::ARITY_TYPE_MISMATCH);
                        }
                    }
                    catch (...)
                    {
                        // Exception::terminate(Exception::ErrorType::STACK_VAL_TYPE_MISMATCH);
                    }
                }
            }
        }
        void validateTypeWithFuncIdx(const func_type_t &type, Runtime::index_t funcIdx)
        {
            const auto &modFuncTypes = rtIns->module->funcTypes;
            const auto &funcType = modFuncTypes.at(rtIns->module->funcTypesIndices.at(funcIdx));
            if (funcType != type)
            {
                // Exception::terminate(Exception::ErrorType::FUNC_TYPE_MISMATCH);
            }
        }
        auto collectArities()
        {
            /* Using `std::vector` here for future use. */
            auto returnArityTypes = std::vector<uint8_t>{};
            const auto returnTypeByte = decodeByteFromPC(); // at most one.
            if (static_cast<SectionType>(returnTypeByte) != SectionType::result)
            {
                returnArityTypes.push_back(returnTypeByte);
            }
            return returnArityTypes;
        }
        template <typename T>
        uint8_t *retFromFrameWithCont(uint32_t depth = 0)
        {
            if constexpr (std::is_same_v<T, Runtime::RTActivFrame>)
            {
                const auto &frameOffset = refTrackedTopFrameByType(Runtime::STVariantIndex::ACTIVATION);
                const auto &frame = std::get<T>(*frameOffset.ptr);
                const auto &returnArity = frame.returnArity;
                const auto cont = frame.cont;
                validateArity(*returnArity);
                eraseRangeFromStack(frameOffset.offset, returnArity->size());
                eraseFromFrameBitmap(Runtime::STVariantIndex::LABEL, depth);
                eraseFromFrameBitmap(Runtime::STVariantIndex::ACTIVATION, 1);
                labelAboveActivFrameCount -= depth;
                return cont;
            }
            if constexpr (std::is_same_v<T, Runtime::RTLabelFrame>)
            {
                // Idx starts from zero.
                const auto &frameOffset = refTrackedTopFrameByType(Runtime::STVariantIndex::LABEL, depth);
                const auto &frame = std::get<T>(*frameOffset.ptr);
                const auto &returnArity = frame.returnArity;
                const auto cont = frame.cont;
                validateArity(returnArity); // May throw.
                eraseRangeFromStack(frameOffset.offset, returnArity.size());
                eraseFromFrameBitmap(Runtime::STVariantIndex::LABEL, depth + 1); // Erase count.
                labelAboveActivFrameCount -= (depth + 1);
                return cont;
            }
        }
        auto parseBrTableInfo()
        {
            std::vector<uint32_t> brTableEntries = {};
            const auto targetCount = decodeVaruintFromPC<Runtime::imme_u32_t>();
            for (auto i = 0; i <= targetCount; ++i)
            {
                brTableEntries.push_back(decodeVaruintFromPC<Runtime::imme_u32_t>()); // entries.
            }
            return brTableEntries;
        }
        MemImme parseMemImmeInfo()
        {
            const auto flags = decodeVaruintFromPC<Runtime::imme_u32_t>();
            const auto offset = decodeVaruintFromPC<Runtime::imme_u32_t>();
            return {flags, offset};
        }
        size_t resizeMem(int32_t pages, uint32_t memIdx = 0)
        {
            if (rtIns->rtMems.size() > 0)
            {
                auto &rtMem = rtIns->rtMems.at(memIdx);
                const auto totalPages = rtMem.size + pages;
                if (totalPages <= WASM_MAX_PAGES && (rtMem.maximumPages == 0 || totalPages <= rtMem.maximumPages))
                {
                    const size_t totalBytes = totalPages * WASM_PAGE_SIZE_IN_BYTE;
                    const auto ptr = static_cast<uint8_t *>(std::realloc(rtMem.ptr, totalBytes));
                    if (ptr)
                    {
                        const auto prevPages = rtMem.size;
                        const auto prevBytes = prevPages * WASM_PAGE_SIZE_IN_BYTE;
                        std::memset(ptr + prevBytes, 0, totalBytes - prevBytes);
                        rtMem.ptr = ptr;
                        rtMem.size = totalPages;
                        return prevPages;
                    }
                    else
                    {
                        return -1;
                    }
                }
                else
                {
                    return -1;
                }
            }
            else
            {
                return -1;
            }
        }
        // (T, T) -> U.
        template <typename T, typename U>
        void opHandlerFTRO(std::function<U(T, T)> handler)
        {
            try
            {
                auto &x = std::get<Runtime::RTValueFrame>(rtIns->stack.back());                      // "c2".
                auto &y = std::get<Runtime::RTValueFrame>(rtIns->stack.at(rtIns->stack.size() - 2)); // "c1".
                auto ret = handler(std::get<T>(y.value), std::get<T>(x.value));
                rtIns->stack.pop_back(); // Keep "c1" on the stage.
                y.value = ret;
            }
            catch (...)
            {
                // Exception::terminate(Exception::ErrorType::STACK_VAL_TYPE_MISMATCH);
            }
        }
        // (T) -> U.
        template <typename T, typename U>
        void opHandlerFORO(std::function<U(T)> handler)
        {
            try
            {
                auto &v = std::get<Runtime::RTValueFrame>(rtIns->stack.back());
                auto ret = handler(std::get<T>(v.value));
                v.value = ret;
            }
            catch (...)
            {
                // Exception::terminate(Exception::ErrorType::STACK_VAL_TYPE_MISMATCH);
            }
        }
        engine_result_t postProcess();
        engine_result_t execute(std::optional<uint32_t> = {});
    };

    struct Interpreter
    {
        using op_handler_proto_t = void (*)(Executor &, std::optional<uint32_t>);
        using op_handler_info_t = std::optional<uint32_t>;
        static std::array<op_handler_proto_t, sizeof(uint8_t) * 1 << 8> opTokenHandlers;
        ITERATE_ALL_OPCODE(DECLARE_OPCODE_HANDLER)
    };
}

#endif //  SRC_EXECUTOR_H_