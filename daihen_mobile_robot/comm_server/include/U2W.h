/******************************************************************************/
class U2W
{
private:
	WCHAR*	p;
public:
	U2W(const char* src)
	{
		extern WCHAR* __from_utf8_to_w(const char* pUtf8String, int nBytes);
		p = __from_utf8_to_w(src, -1);
	}
	U2W(const char* src, int nLength)
	{
		extern WCHAR* __from_utf8_to_w(const char* pUtf8String, int nBytes);
		p = __from_utf8_to_w(src, nLength);
	}
	virtual ~U2W()
	{
		delete [] p;
	}
	operator LPCWSTR()
	{
		return(p);
	}
};

/******************************************************************************/
class W2U
{
private:
	char*	p;
public:
	W2U(LPCWSTR src)
	{
		int n = (int)wcslen(src) * 6 + 1;
		p = new char[n];
		extern int __to_utf8(char* pUtf8String, LPCWSTR pText);
		__to_utf8(p, src);
	}
	virtual ~W2U()
	{
		if( p )
			delete [] p;
	}
	operator LPCSTR()
	{
		return(p);
	}
	operator ByteArray()
	{
		int n = strlen(p);
		ByteArray a;
		a.SetSize(n);
		if( n > 0 )
			memcpy((char*)a.GetData(), p, strlen(p));
		return a;
	}
};
