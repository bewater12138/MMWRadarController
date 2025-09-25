#include "code_convert.h"
#include <Windows.h>
#include <string>

#pragma warning(disable: 4267)

static __declspec(thread) std::string		ansi_buf;
static __declspec(thread) std::wstring		wchar_buf;
static __declspec(thread) U8String			u8_buf;

size_t	Widechar_To_ANSI(const wchar_t* wstr);
size_t	Widechar_To_ANSI(const wchar_t* wstr, size_t len);
size_t	Widechar_To_UTF8(const wchar_t* wstr);
size_t	Widechar_To_UTF8(const wchar_t* wstr, size_t len);
size_t	ANSI_To_Widechar(const char* str);
size_t	ANSI_To_Widechar(const char* str, size_t len);
size_t	UTF8_To_Widechar(const CHAR8* str);
size_t	UTF8_To_Widechar(const CHAR8* str, size_t len);

const CHAR8* ANSIToUTF8(const char* str, size_t len)
{
	size_t wchar_len = ANSI_To_Widechar(str, len);
	Widechar_To_UTF8(wchar_buf.c_str(), wchar_len);
	return u8_buf.c_str();
}

const char* UTF8ToANSI(const CHAR8* str, size_t len)
{
	size_t wchar_len = UTF8_To_Widechar(str, len);
	Widechar_To_ANSI(wchar_buf.c_str(), wchar_len);
	return ansi_buf.c_str();
}

size_t Widechar_To_ANSI(const wchar_t* wstr)
{
	size_t wstr_size = std::wstring_view(wstr).size();
	return Widechar_To_ANSI(wstr, wstr_size);
}
size_t Widechar_To_ANSI(const wchar_t* wstr, size_t len)
{
	size_t need_size = len * 3;
	ansi_buf.reserve(need_size + 1);
	int write_byte = WideCharToMultiByte(CP_ACP, 0, wstr, len, ansi_buf.data(), need_size, "?", NULL);
	ansi_buf.data()[write_byte] = 0;
	return write_byte;
}

size_t ANSI_To_Widechar(const char* str)
{
	size_t str_size = std::strlen(str);
	return ANSI_To_Widechar(str, str_size);
}
size_t ANSI_To_Widechar(const char* str, size_t len)
{
	size_t need_size = len;
	wchar_buf.reserve(need_size + 1);
	int write_char = MultiByteToWideChar(CP_ACP, 0, str, len, wchar_buf.data(), need_size);
	wchar_buf.data()[write_char] = 0;
	return write_char;
}

size_t Widechar_To_UTF8(const wchar_t* wstr)
{
	size_t wstr_size = std::wstring_view(wstr).size();
	return Widechar_To_UTF8(wstr, wstr_size);
}
size_t Widechar_To_UTF8(const wchar_t* wstr, size_t len)
{
	size_t need_size = len * 3;
	u8_buf.reserve(need_size + 1);
	int write_byte = WideCharToMultiByte(CP_UTF8, 0, wstr, len, (char*)u8_buf.data(), need_size, "?", NULL);
	u8_buf.data()[write_byte] = 0;
	return write_byte;
}

size_t UTF8_To_Widechar(const CHAR8* str)
{
	size_t str_size = std::strlen((const char*)str);
	return UTF8_To_Widechar(str, str_size);
}
size_t UTF8_To_Widechar(const CHAR8* str, size_t len)
{
	size_t need_size = len;
	wchar_buf.reserve(need_size + 1);
	int write_char = MultiByteToWideChar(CP_UTF8, 0, (char*)str, len, wchar_buf.data(), need_size);
	wchar_buf.data()[write_char] = 0;
	return write_char;
}

#pragma warning(default: 4267)