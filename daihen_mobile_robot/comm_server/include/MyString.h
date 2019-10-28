#ifndef __MyString_H
#define __MyString_H

/******************************************************************************/
#define CLASSNAME String1
#define ARRAYNAME String1Array
#define MAPNAME String1Map
#define BASE std::string
#define CHARTYPE char
#define CHARSIZE 1
#include "MyStringCore.h"
#undef CLASSNAME
#undef ARRAYNAME
#undef MAPNAME
#undef BASE
#undef CHARTYPE
#undef CHARSIZE
extern String1 Edit1(const char* pFormat, ...);
extern String1 EditComma1(int n);

/******************************************************************************/
#define CLASSNAME String2
#define ARRAYNAME String2Array
#define MAPNAME String2Map
#define BASE std::wstring
#define CHARTYPE wchar_t
#define CHARSIZE 2
#include "MyStringCore.h"
#undef CLASSNAME
#undef ARRAYNAME
#undef MAPNAME
#undef BASE
#undef CHARTYPE
#undef CHARSIZE
extern String2 Edit2(const wchar_t* pFormat, ...);
extern String2 EditComma2(int n);

/******************************************************************************/
#ifndef _UNICODE
typedef	String1			String;
typedef	String1Array	StringArray;
typedef	String1Map		StringMap;
#define Edit			Edit1
#define EditComma		EditComma1
#else
typedef	String2			String;
typedef	String2Array	StringArray;
typedef	String2Map		StringMap;
#define Edit			Edit2
#define EditComma		EditComma2
#endif

#endif
