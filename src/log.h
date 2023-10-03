#ifndef SRC_LOG_H_
#define SRC_LOG_H_

#include <iostream>

#define LOG(...) std::cout << "[WVM]: " << (__VA_ARGS__) << std::endl

#endif // SRC_LOG_H_