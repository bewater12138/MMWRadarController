#pragma once
#include <string_view>

#ifdef __cpp_lib_char8_t
using CHAR8 = char8_t;
#define U8String std::u8string
#define U8String_view std::u8string_view
#else
using CHAR8 = char;
#define U8String std::string
#define U8String_view std::string_view
#endif

// ANSI 编码转换至 UTF-8 编码
const CHAR8* ANSIToUTF8(const char* str, size_t len);
inline const CHAR8* ANSIToUTF8(std::string_view str) { return ANSIToUTF8(str.data(), str.size()); }
template<size_t Len>
const CHAR8* ANSIToUTF8(const char(&str)[Len]) { return ANSIToUTF8(str, Len - 1); }

// UTF-8 编码转换至 ANSI 编码
const char* UTF8ToANSI(const CHAR8* str, size_t len);
inline const char* UTF8ToANSI(U8String_view str) { return UTF8ToANSI(str.data(), str.size()); }
template<size_t Len>
const char* UTF8ToANSI(const CHAR8(&str)[Len]) { return UTF8ToANSI(str, Len - 1); }

#ifdef __cpp_lib_char8_t
// const char* 到 const char8_t* 的隐式转换

inline const char* UTF8ToANSI(const char* str, size_t len)
{
	return UTF8ToANSI((const CHAR8*)str, len);
}
inline const char* UTF8ToANSI(std::string_view str)
{
	return UTF8ToANSI(str.data(), str.size());
}
template<size_t Len>
const char* UTF8ToANSI(const char(&str)[Len])
{
	return UTF8ToANSI(str, Len);
}
#endif