#include "instance.h"

namespace wvm
{
    Instance::Instance(std::shared_ptr<Module> module_ptr) : module_ptr(module_ptr)
    {
    }

    Instance::~Instance()
    {
    }

    void Instance::instantiate(){

    };
}