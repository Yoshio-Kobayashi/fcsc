/******************************************************************************/
#include <libcpp.h>

/******************************************************************************/
// ("ABC  DEF", " ") -> [2]{"ABC","DEF"}
String1Array String1::split(const String1& seps) const
{
	String1Array result;

	char* pBuf = new char[length() + 8];
	strcpy(pBuf, c_str());

#ifdef _WINDOWS
	for( char* p=strtok(pBuf, seps) ; p ; p=strtok(NULL, seps) )
		result.Add(p);
#else
	char* svp = NULL;
	for( char* p=strtok_r(pBuf, seps, &svp) ; p ; p=strtok_r(NULL, seps, &svp) )
		result.Add(p);
#endif
	delete [] pBuf;

	if( result.size() == 0 )
		result.Add("");

	return result;
}
String2Array String2::split(const String2& seps) const
{
	String2Array result;

	wchar_t* pBuf = new wchar_t[length() + 8];
	wcscpy(pBuf, c_str());

#ifdef _WINDOWS
	for( wchar_t* p=wcstok(pBuf, seps) ; p ; p=wcstok(NULL, seps) )
		result.Add(p);
#else
	wchar_t* svp = NULL;
	for( wchar_t* p=wcstok(pBuf, seps, &svp) ; p ; p=wcstok(NULL, seps, &svp) )
		result.Add(p);
#endif
	delete [] pBuf;

	if( result.size() == 0 )
		result.Add(L"");

	return result;
}

/******************************************************************************/
// ("ABC  DEF", " ") -> [3]{"ABC","","DEF"}
String1Array String1::splitExact(const String1& sep) const
{
	String1Array result;
	String1::size_type start = 0;
	String1::size_type nSrcLen = length();
	String1::size_type nSepLen = sep.length();
	while( true )
	{
		String1::size_type end = find(sep, start);

		if( end != String1::npos )
		{
			result.push_back(substr(start, end - start));
		}
		else
		{
			result.push_back(substr(start, nSrcLen - start));
			break;
		}
		start = end + nSepLen;
	}
	return result;
}
String2Array String2::splitExact(const String2& sep) const
{
	String2Array result;
	String2::size_type start = 0;
	String2::size_type nSrcLen = length();
	String2::size_type nSepLen = sep.length();
	while( true )
	{
		String2::size_type end = find(sep, start);

		if( end != String2::npos )
		{
			result.push_back(substr(start, end - start));
		}
		else
		{
			result.push_back(substr(start, nSrcLen - start));
			break;
		}
		start = end + nSepLen;
	}
	return result;
}

/******************************************************************************/
void String1Array::Sort(bool bAsc/*=true*/)
{
	for( int ia=0 ; ia<GetSize() ; ia++ )
	{
		for( int ib=ia+1 ; ib<GetSize() ; ib++ )
		{
			String1 a = at(ia);
			String1 b = at(ib);
			if(	(bAsc && (_stricmp(a, b) > 0)) || (!bAsc && (_stricmp(a, b) < 0)) )
			{
				(*this)[ia] = b;
				(*this)[ib] = a;
			}
		}
	}
}
void String2Array::Sort(bool bAsc/*=true*/)
{
	for( int ia=0 ; ia<GetSize() ; ia++ )
	{
		for( int ib=ia+1 ; ib<GetSize() ; ib++ )
		{
			String2 a = at(ia);
			String2 b = at(ib);
			if(	(bAsc && (_wcsicmp(a, b) > 0)) || (!bAsc && (_wcsicmp(a, b) < 0)) )
			{
				(*this)[ia] = b;
				(*this)[ib] = a;
			}
		}
	}
}

/******************************************************************************/
String1 String1::replace(const char* pOld, const char* pNew) const
{
	std::string s1 = c_str();
	std::string s2 = pOld;
	std::string s3 = pNew;

	for( std::string::size_type pos=s1.find(s2) ; pos!=std::string::npos ; )
    {
        s1.replace(pos, s2.length(), s3);
        pos = s1.find(s2, pos + s3.length());
    }
    return String1(s1.c_str());
}
String2 String2::replace(const wchar_t* pOld, const wchar_t* pNew) const
{
	std::wstring s1 = c_str();
	std::wstring s2 = pOld;
	std::wstring s3 = pNew;

	for( std::wstring::size_type pos=s1.find(s2) ; pos!=std::wstring::npos ; )
    {
        s1.replace(pos, s2.length(), s3);
        pos = s1.find(s2, pos + s3.length());
    }
    return String2(s1.c_str());
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
String1 Edit1(const char* pFormat, ...)
{
	char buf[1024];

	va_list args;
	va_start(args, pFormat);
	_vsnprintf(buf, sizeof(buf), pFormat, args);
	va_end(args);
	return String1(buf);
}
String2 Edit2(const wchar_t* pFormat, ...)
{
	wchar_t buf[1024];

	va_list args;
	va_start(args, pFormat);
	_vsnwprintf(buf, sizeof(buf), pFormat, args);
	va_end(args);
	return String2(buf);
}

/******************************************************************************/
String1 EditComma1(int n)
{
	if( n < 1000 )
		return(Edit1("%d", n));
	if( n < 1000000 )
		return(Edit1("%d,%03d", n/1000, n%1000));
	if( n < 1000000000 )
		return(Edit1("%d,%03d,%03d", n/1000000, (n/1000)%1000, n%1000));
	return(Edit1("%d,%03d,%03d,%03d", n/1000000000, (n/1000000)%1000, (n/1000)%1000, n%1000));
}
String2 EditComma2(int n)
{
	if( n < 1000 )
		return(Edit2(L"%d", n));
	if( n < 1000000 )
		return(Edit2(L"%d,%03d", n/1000, n%1000));
	if( n < 1000000000 )
		return(Edit2(L"%d,%03d,%03d", n/1000000, (n/1000)%1000, n%1000));
	return(Edit2(L"%d,%03d,%03d,%03d", n/1000000000, (n/1000000)%1000, (n/1000)%1000, n%1000));
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
String1 String1Array::Join(const char* pSep)
{
	String1 buf = "";
	for( int i=0 ; i<GetSize() ; i++ )
	{
		if( i )
			buf += pSep;
		buf += (*this)[i];
	}
	return buf;
}
String2 String2Array::Join(const wchar_t* pSep)
{
	String2 buf = L"";
	for( int i=0 ; i<GetSize() ; i++ )
	{
		if( i )
			buf += pSep;
		buf += (*this)[i];
	}
	return buf;
}
