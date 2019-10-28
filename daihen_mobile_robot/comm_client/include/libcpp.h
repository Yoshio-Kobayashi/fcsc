#ifndef __Libcpp_H
#define __Libcpp_H

/******************************************************************************/
#if defined(WIN32) || defined(WIN64)
  #ifndef _WINDOWS
    #define _WINDOWS
  #endif // _WINDOWS
#endif // defined(WIN32) || defined(WIN64)

#ifdef _WINDOWS

  #define _CRT_SECURE_NO_WARNINGS
  #define _CRT_NON_CONFORMING_SWPRINTFS

  #ifndef WINVER
    #define WINVER	0x500
  #endif // WINVER

  #include <afxwin.h>
  #include <io.h>
  #include <winsock2.h>
  #pragma comment(lib, "ws2_32.lib")
  #include <iphlpapi.h>
  #pragma comment(lib, "iphlpapi.lib")
  #include <process.h>
  #include <pthread.h>
  #include <signal.h>
  #include <limits.h>

#else // _WINDOWS
  #include <stdio.h>
  #include <stdlib.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <string.h>
  #include <time.h>
  #include <sys/types.h>
  ///#include <gst/gst.h>
  #include <pthread.h>
  #include <sys/ipc.h>
  #include <sys/sem.h>
  #include <sys/shm.h>
  #include <sys/msg.h>
  #include <errno.h>
  #include <sys/select.h>
  #include <sys/time.h>
  #include <sys/select.h>
  #include <sys/stat.h>
  #include <dirent.h>
  #include <stdarg.h>
  #include <termios.h>
  #include <signal.h>
  #include <limits.h>
  #include <sys/socket.h>
  #include <arpa/inet.h>
  #include <netdb.h>
  #include <sys/ioctl.h>
#endif // _WINDOWS
  #include <math.h>

/******************************************************************************/
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <functional>
using namespace std;

/******************************************************************************/
typedef unsigned char			BYTE;
typedef unsigned short			WORD;
typedef unsigned long			DWORD;
typedef unsigned long long		QWORD;

#ifdef _WINDOWS
  typedef __int64				INT64;
  #ifdef _UNICODE
    #define	_ttof		_wtof
  #else // _UNICODE
    #define	_ttof		atof
  #endif // _UNICODE
#else // _WINDOWS
  typedef long long				INT64;

  typedef int					HANDLE;
  #define INVALID_HANDLE_VALUE	(-1)

  typedef char					CHAR;
  typedef wchar_t				WCHAR;

  typedef char*					LPSTR;
  typedef WCHAR*				LPWSTR;
  typedef const CHAR*			LPCSTR;
  typedef const WCHAR*			LPCWSTR;

  typedef CHAR					TCHAR;
  typedef TCHAR*				LPTSTR;
  typedef const TCHAR*			LPCTSTR;

  typedef int					BOOL;
  #define TRUE					(1)
  #define FALSE					(0)

  ///#define	_T(X)		L#X
  #define	_T(X)		X

  #define _tcscpy		strcpy
  #define _tcslen		strlen
  #define _fgetts		fgets
  #define _fputts		fputs
  #define _vsntprintf	vsnprintf
  #define _ttoi			atoi
  #define _ttof			atof
  #define _tfopen		fopen
  #define _stricmp		strcasecmp
  #define _wcsicmp		wcscasecmp
  #define _vsnprintf	vsnprintf
  #define _vsnwprintf	vswprintf
  #define _tcstoul		strtoul
  #define _unlink		unlink
  #define _tunlink		unlink
  #define _stscanf		scanf

#endif // _WINDOWS

/******************************************************************************/
#define countof(X)	(sizeof(X) / sizeof(X[0]))

#include <Array.h>

class ByteArray : public Array<BYTE>
{
public:
	ByteArray()
	{
	}

	ByteArray(int n, void** ppBuffer=NULL)
	{
		SetSize(n);
		if( ppBuffer )
			*ppBuffer = GetData();
	}

	ByteArray& operator=(const ByteArray& src)
	{
		Copy(src);
		return *this;
	}

	ByteArray operator+(const ByteArray& src) const
	{
		ByteArray tmp;
		tmp.Copy(*this);
		tmp.Append(src);
		return tmp;
	}

	ByteArray& operator+=(const ByteArray& src)
	{
		Append(src);
		return *this;
	}

	ByteArray sub(int s, int n) const
	{
		ByteArray tmp;
		for( int i=s ; i<(s + n) ; i++ )
			tmp.Add(at(i));
		return tmp;
	}

	ByteArray sub(int s) const
	{
		return sub(s, GetSize() - s);
	}
};

#include <Map.h>
#include <MyString.h>

#ifdef _WINDOWS
  #ifdef _UNICODE
    #include <M2W.h>
    #include <U2W.h>
  #endif // _UNICODE
#endif // _WINDOWS

#ifndef _UNICODE
  #define T2M(X)	X
#endif // _UNICODE

#include <Variant.h>
#include <Parameters.h>
#include <Mutex.h>
#include <ScopeLock.h>
#include <Queue.h>
#include <SharedMemory.h>
#include <MessageQue.h>
#include <File.h>
#include <IniFile.h>
#include <Thread.h>
#include <EscapeSequence.h>
#include <Console.h>

#include <Socket.h>

#include <String1Holder.h>
#include <String2Holder.h>
#ifdef _UNICODE
  typedef String2Holder		StringHolder;
#else // _UNICODE
  typedef String1Holder		StringHolder;
#endif // _UNICODE

// 3d
#define PI			(3.1415927f)
#define	DEG2RAD		(PI/180.0)
#define	RAD2DEG		(180.0/PI)

// ifndef STEP1_BOARD
#include <Vector3.h>
#include <Quat.h>

#endif // __Libcpp_H
