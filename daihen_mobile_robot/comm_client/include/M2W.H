/******************************************************************************/
class M2W
{
private:
	WCHAR*	p;
public:
	M2W(LPCSTR src)
	{
		int n = (int)strlen(src) + 1;
		p = new WCHAR[n];
		MultiByteToWideChar(CP_ACP, 0, src, -1, p, n);
	}
	virtual ~M2W()
	{
		delete [] p;
	}
	operator LPCWSTR() const
	{
		return(p);
	}
	LPCWSTR c_str() const
	{
		return p;
	}
};

/******************************************************************************/
class W2M
{
private:
	CHAR*	p;
public:
	W2M(LPCWSTR src)
	{
		int n = (int)wcslen(src) * 2 + 1;
		p = new CHAR[n];
		WideCharToMultiByte(CP_ACP, 0, src, -1, p, n, NULL, NULL);
	}
	virtual ~W2M()
	{
		delete [] p;
	}
	operator LPCSTR() const
	{
		return(p);
	}
	LPCSTR c_str() const
	{
		return p;
	}
};

/******************************************************************************/
#ifdef _MBCS
#define	TOM(X)	(X)
#define	TOW(X)	M2W(X)
#define	T2M(X)	W2M(X)
#define	M2T(X)	M2W(X)
#define	T2W(X)	(X)
#define	W2T(X)	(X)
#else
#define	TOM(X)	W2M(X)
#define	TOW(X)	(X)
#define	T2M(X)	W2M(X)
#define	M2T(X)	M2W(X)
#define	T2W(X)	(X)
#define	W2T(X)	(X)
#endif
