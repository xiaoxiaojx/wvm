#ifndef SRC_INSTANCE_H_
#define SRC_INSTANCE_H_

#include "module.h"

namespace wvm
{
    class Instance
    {
    public:
        Instance(std::shared_ptr<Module>);
        ~Instance();

        void instantiate();

        std::shared_ptr<Module> module_ptr;
    };
}

#endif // SRC_INSTANCE_H_