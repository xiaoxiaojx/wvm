

#include "executor.h"
#include "interpreter.h"
#include "log.h"
namespace wvm
{

    Executor::Executor(std::shared_ptr<Runtime> rtIns) : rtIns(rtIns)
    {
    }

    Executor::Executor(uint8_t *pc, shared_module_runtime_t rtIns) : pc(pc), rtIns(rtIns) {}

    Executor::engine_result_t Executor::execute(std::optional<uint32_t> invokeIdx)
    {
        LOG("-> execute");
        if (!invokeIdx.has_value() && rtIns->rtEntryIdx.has_value())
        {
            invokeIdx = *rtIns->rtEntryIdx;
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
}