#ifndef SRC_DECODE_H_
#define SRC_DECODE_H_

#include <string>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "decoder.h"

namespace wvm
{
    class Decoder
    {
    private:
        std::ifstream &readable_;

    public:
        Decoder(std::ifstream &readable);
        ~Decoder();

        std::ifstream &readable();

        std::vector<uint8_t> readBytes(uint8_t n);
        uint8_t readByte();
        std::string readStringByBytes(size_t n);

        template <typename U>
        static std::vector<uint8_t> retrievePackedLEB128Bytes(U &&in)
        {
            std::vector<uint8_t> v = {};
            while (true)
            {
                uint8_t byte;
                if constexpr (std::is_same_v<typename std::decay<U>::type, uint8_t *>)
                {
                    byte = *in++;
                }
                else
                {
                    byte = static_cast<uint8_t>(in.get());
                }
                v.emplace_back(byte);
                if (!(byte & 0x80))
                {
                    break;
                }
            }
            return v;
        }

        template <typename T>
        static std::vector<uint8_t> encodeVaruint(T in)
        {
            std::vector<uint8_t> v;
            do
            {
                uint8_t byte = in & 0x7f;
                in >>= 7;
                if (in != 0)
                    byte |= 0x80;
                v.push_back(byte);
            } while (in != 0);
            return v;
        }
        template <typename T, typename U>
        static T decodeVaruint(U &&in)
        {
            const auto v = retrievePackedLEB128Bytes(
                std::forward<U>(in));
            T val = 0;
            unsigned shift = 0;
            for (auto byte : v)
            {
                val |= (static_cast<T>(byte & 0x7f) << shift);
                shift += 7;
            }
            return val;
        }
        template <typename T, typename U>
        static T decodeVarint(U &&in)
        {
            const auto v = retrievePackedLEB128Bytes(
                std::forward<U>(in));
            T val = 0;
            unsigned shift = 0;
            uint8_t b = 0;
            for (auto byte : v)
            {
                b = byte;
                val |= (static_cast<T>(byte & 0x7f) << shift);
                shift += 7;
            }
            if ((shift < sizeof(T) * 8) && (b & 0x40))
            {
                // Sign extend.
                val |= (~0 << shift);
            }
            return val;
        }

#define WALK_FUNC_DEF(name, type, suffix)          \
    type read##name()                              \
    {                                              \
        return decodeVar##suffix<type>(readable_); \
    }

#define DEFINE_WALK_FUNCS(V) \
    V(U8, uint8_t, uint)     \
    V(U16, uint16_t, uint)   \
    V(U32, uint32_t, uint)   \
    V(I8, int8_t, int)       \
    V(I16, int16_t, int)     \
    V(I32, int32_t, int)

        DEFINE_WALK_FUNCS(WALK_FUNC_DEF)
    };
}

#endif // SRC_DECODE_H_