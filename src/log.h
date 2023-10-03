#ifndef SRC_LOG_H_
#define SRC_LOG_H_

#include <iostream>

// 重载输出运算符以支持 std::vector<char> 类型的输出
std::ostream &operator<<(std::ostream &os, const std::vector<char> &vec)
{
    for (const auto &ch : vec)
    {
        os << ch;
    }
    return os;
}

#define LOG(...) std::cout << "[WVM]: " << (__VA_ARGS__) << std::endl

#endif // SRC_LOG_H_