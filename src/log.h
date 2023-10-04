#ifndef SRC_LOG_H_
#define SRC_LOG_H_

#include <iostream>
#include <vector>
#include <sstream>

namespace wvm
{
    // 打印整数类型的数据
    template <typename T>
    inline void logValue(const T &value, std::ostream &os)
    {
        os << value;
    }

    // 打印字符类型的数据
    inline void logValue(char value, std::ostream &os)
    {
        os << "'" << value << "'";
    }

    // 打印std::vector<uint8_t>类型的数据
    inline void logValue(const std::vector<uint8_t> &value, std::ostream &os)
    {
        os << "[ ";
        for (const auto &item : value)
        {
            os << static_cast<int>(item) << " ";
        }
        os << "]";
    }

    // 可变参数模板，用于处理不同类型的参数
    template <typename T>
    inline void logValues(std::ostream &os, const T &value)
    {
        logValue(value, os);
    }

    template <typename T, typename... Args>
    inline void logValues(std::ostream &os, const T &value, const Args &...args)
    {
        logValue(value, os);
        logValues(os, args...);
    }
}

// 定义LOG宏
#define LOG(...)                             \
    do                                       \
    {                                        \
        std::ostringstream oss;              \
        wvm::logValues(oss, __VA_ARGS__);    \
        std::cout << oss.str() << std::endl; \
    } while (0)

#endif // SRC_LOG_H_