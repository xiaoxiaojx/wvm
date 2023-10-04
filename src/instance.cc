#include "instance.h"

namespace wvm
{
    Instance::Instance(std::shared_ptr<Module> module_ptr) : module_ptr(module_ptr)
    {
    }

    Instance::~Instance()
    {
    }

    std::shared_ptr<wvm::Runtime> Instance::instantiate()
    {
        std::shared_ptr<wvm::Runtime> runtime_ptr = std::make_shared<wvm::Runtime>(module_ptr);
        /* func */
        for (auto i = 0; i < module_ptr->funcDefs.size(); ++i)
        {
            const auto typeIdx = module_ptr->funcTypesIndices.at(i);
            const auto &funcType = module_ptr->funcTypes.at(typeIdx);
            const auto codeEntry = module_ptr->funcDefs.at(i).body.data();
            // Wasm types -> RT types (value).
            runtime_ptr->rtFuncDescriptor.emplace_back(&funcType, codeEntry);

            runtime_ptr->expandWasmTypesToRTValues(
                runtime_ptr->rtFuncDescriptor.back().localsDefault,
                // 参数类型
                funcType.first,
                // 内部变量
                module_ptr->funcDefs.at(i).locals);
        }

        /* entry point */
        const auto entryFunc = std::find_if(
            module_ptr->exports.begin(),
            module_ptr->exports.end(),
            [](auto &item) -> bool
            {
                return item.name == "add" && item.extKind == EXT_KIND_FUNC;
            });

        const auto funcIdx = entryFunc->extIdx;
        if (entryFunc != module_ptr->exports.end())
        {
            runtime_ptr->rtEntryIdx = funcIdx;
        }
        const auto &inputFuncArgTypes = runtime_ptr->rtFuncDescriptor.at(funcIdx).funcType->first;

        runtime_ptr->stack.push_back(
            Runtime::RTValueFrame{
                runtime_ptr->convertStrToRTVal("1", inputFuncArgTypes[0])});
        runtime_ptr->stack.push_back(
            Runtime::RTValueFrame{
                runtime_ptr->convertStrToRTVal("2", inputFuncArgTypes[1])});
        return runtime_ptr;
    };

}