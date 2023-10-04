#include <string>
#include <fstream>
#include <iostream>

#include "decoder.h"

namespace wvm
{
    Decoder::Decoder(std::ifstream &readable) : readable_(readable)
    {
    }

    Decoder::~Decoder()
    {
    }

    std::ifstream &Decoder::readable()
    {
        return readable_;
    }

    std::vector<uint8_t> Decoder::readBytes(uint8_t n)
    {
        std::vector<uint8_t> buffer(n);                             // 创建一个存储字节的缓冲区
        readable_.read(reinterpret_cast<char *>(buffer.data()), n); // 从文件流中读取 n 个字节到缓冲区

        // 如果未能读取足够的字节，可以根据需要进行处理（例如抛出异常）
        if (readable_.gcount() < static_cast<std::streamsize>(n))
        {
            std::cerr << "Error: Unable to read " << n << " bytes from the file." << std::endl;
        }

        return buffer; // 返回读取到的字节数据
    }

    uint8_t Decoder::readByte()
    {
        return static_cast<uint8_t>(readable_.get());
    }

    std::string Decoder::readStringByBytes(size_t n)
    {
        char str[n];
        readable_.read(str, n);
        return std::string(str, n);
    }
}