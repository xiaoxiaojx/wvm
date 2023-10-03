#ifndef SRC_INSTANCE_H_
#define SRC_INSTANCE_H_

#include "module.h"
#include <optional>

constexpr uint8_t EXT_KIND_FUNC = 0x0;

namespace wvm
{
    struct Runtime
    {
        enum class STVariantIndex : int8_t
        {
            // The order of types here should be consistent with the below stack definition, -
            // and this will be used to test the type of the stack frame.
            VALUE = 0,
            LABEL,
            ACTIVATION,
        };
        using rt_i32_t = int32_t;
        using rt_i64_t = int64_t;
        using rt_f32_t = float;
        using rt_f64_t = double;
        using imme_u32_t = uint32_t;
        using imme_u64_t = uint64_t;
        using relative_depth_t = uint32_t;
        using index_t = uint32_t;
        using runtime_value_t = std::variant<rt_i32_t, rt_i64_t, rt_f32_t, rt_f64_t>;
        struct RTFuncDescriptor
        {
            SET_STRUCT_MOVE_ONLY(RTFuncDescriptor)
            const func_type_t *funcType;
            uint8_t *codeEntry;
            std::vector<runtime_value_t> localsDefault;
            RTFuncDescriptor(const func_type_t *funcType, uint8_t *codeEntry)
                : funcType(funcType), codeEntry(codeEntry) {}
        };
        struct RTValueFrame
        {
            SET_STRUCT_MOVE_ONLY(RTValueFrame)
            runtime_value_t value;
            RTValueFrame(const runtime_value_t &value)
                : value(value) {}
        };
        struct RTLabelFrame
        {
            SET_STRUCT_MOVE_ONLY(RTLabelFrame)
            uint8_t *cont;
            type_seq_t returnArity;
            RTLabelFrame(uint8_t *cont, const type_seq_t &returnArity = {})
                : cont(cont), returnArity(returnArity) {}
        };
        struct RTActivFrame
        {
            SET_STRUCT_MOVE_ONLY(RTActivFrame)
            std::vector<runtime_value_t> locals;
            uint8_t *cont;
            const type_seq_t *returnArity;
            RTActivFrame(std::vector<runtime_value_t> &locals, uint8_t *cont, const type_seq_t *returnArity)
                : locals(locals), cont(cont), returnArity(returnArity) {}
        };
        struct RTMemHolder
        {
            SET_STRUCT_MOVE_ONLY(RTMemHolder)
            size_t size; // Pages.
            uint8_t *ptr;
            uint32_t maximumPages;
            RTMemHolder(size_t size, uint8_t *ptr, uint32_t maximumPages)
                : size(size), ptr(ptr), maximumPages(maximumPages) {}
        };
        shared_module_t module;
        std::vector<RTMemHolder> rtMems;
        std::vector<std::vector<std::optional<uint32_t>>> rtTables; // Func idx inside.
        std::vector<runtime_value_t> rtGlobals;
        using stack_frame_t = std::variant<RTValueFrame, RTLabelFrame, RTActivFrame>;
        std::vector<stack_frame_t> stack;
        std::optional<uint32_t> rtEntryIdx;
        std::vector<RTFuncDescriptor> rtFuncDescriptor;
        explicit Runtime(shared_module_t module) : module(module) {}
        ~Runtime()
        {
            // Free allocated mem.
            std::for_each(rtMems.begin(), rtMems.end(), [](RTMemHolder &mem)
                          { std::free(mem.ptr); });
        }
    };

    class Instance
    {
    public:
        Instance(std::shared_ptr<Module>);
        ~Instance();

        std::shared_ptr<wvm::Runtime> instantiate();

        std::shared_ptr<Module> module_ptr;
    };

    using shared_module_runtime_t = std::shared_ptr<Runtime>;

}

#endif // SRC_INSTANCE_H_