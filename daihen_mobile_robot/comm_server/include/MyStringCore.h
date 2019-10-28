/******************************************************************************/
class CLASSNAME;

/******************************************************************************/
class ARRAYNAME : public Array<CLASSNAME>
{
public:
	ARRAYNAME()
	{
	}
	ARRAYNAME(const ARRAYNAME& src)
	{
		Copy(src);
	}
	void Sort(bool bAsc=true);
	CLASSNAME Join(const CHARTYPE* pSep);
};

/******************************************************************************/
class CLASSNAME : public BASE
{
public:
	CLASSNAME()
	{
	}
	CLASSNAME(const CLASSNAME& src)
	{
		*this = src;
	}
	CLASSNAME& operator=(const CLASSNAME& src)
	{
		*(BASE*)this = src.c_str();
		return *this;
	}
	CLASSNAME& operator=(const CHARTYPE* p)
	{
		*(BASE*)this = p;
		return *this;
	}
	CLASSNAME(const CHARTYPE* src) : BASE(src)
	{
	}
	CLASSNAME(const void* mem, int len)
	{
		CHARTYPE* buffer = new CHARTYPE[len + 1];
		memcpy(buffer, mem, len);
		buffer[len] = '\0';
		*this = buffer;
		delete [] buffer;
	}
	operator const CHARTYPE*() const
	{
		return c_str();
	}
	CLASSNAME operator+(const CHARTYPE* src) const
	{
		BASE buf = *this;
		buf += src;
		return CLASSNAME(buf.c_str());
	}
	CLASSNAME operator+(const CLASSNAME& src) const
	{
		BASE buf = *this;
		buf += src;
		return CLASSNAME(buf.c_str());
	}
	CHARTYPE operator[](int i) const
	{
		return at(i);
	}
	CHARTYPE& operator[](int i)
	{
		return at(i);
	}

	int GetLength() const
	{
		return (int)length();
	}

	CLASSNAME substr(int s, int n) const
	{
		return CLASSNAME(BASE::substr(s, n).c_str());
	}
	CLASSNAME substr(int s) const
	{
		return substr(s, length() - s);
	}
#ifdef STEP1_BOARD
	CLASSNAME left(int n) const
	{
		if( n > GetLength() )
			n = length();
		return substr(0, n);
	}
	CLASSNAME right(int n) const
	{
		if( n > GetLength() )
			n = length();
		return substr(length() - n);
	}
#else
	CLASSNAME left(int n) const
	{
		if( (size_t)n > length() )
			n = length();
		return substr(0, n);
	}
	CLASSNAME right(int n) const
	{
		if( (size_t)n > length() )
			n = length();
		return substr(length() - n);
	}
#endif
	int find(char c, int nStart=0) const
	{
		size_t rc = BASE::find(c, nStart);
		if( rc == ((size_t)-1) )
			return -1;
		return (int)rc;
	}
	int find(const CHARTYPE* p, int nStart=0) const
	{
		size_t rc = BASE::find(p, nStart);
		if( rc == ((size_t)-1) )
			return -1;
		return (int)rc;
	}
	int rfind(char c) const
	{
		size_t rc = BASE::rfind(c);
		if( rc == ((size_t)-1) )
			return -1;
		return (int)rc;
	}
	int rfind(const CHARTYPE* p) const
	{
		size_t rc = BASE::rfind(p);
		if( rc == ((size_t)-1) )
			return -1;
		return (int)rc;
	}
	bool IsWhiteSpace(CHARTYPE _c) const
	{
		unsigned char c = (unsigned char)_c;
		if( c == '\0' )
			return false;
		if( c <= 0x20 )
			return true;
		return false;
	}
	CLASSNAME trimLeft() const
	{
		const CHARTYPE* s = *this;
		const CHARTYPE* p = s;
		while( IsWhiteSpace(*p) )
			p++;
		int n = p - s;
		return substr(n);
	}
	CLASSNAME trimRight() const
	{
		const CHARTYPE* s = *this;
		const CHARTYPE* p = s + length() - 1;
		while( (p >= s) && IsWhiteSpace(*p) )
			p--;
		int n = (p + 1) - s;
		return left(n);
	}
	CLASSNAME trim() const
	{
		return trimLeft().trimRight();
	}

	void toUpper()
	{
		std::transform(BASE::begin(), BASE::end(), BASE::begin(), ::toupper);
	}

	void toLower()
	{
		std::transform(BASE::begin(), BASE::end(), BASE::begin(), ::tolower);
	}

	CLASSNAME upper() const
	{
		CLASSNAME s = *this;
		s.toUpper();
		return s;
	}

	CLASSNAME lower() const
	{
		CLASSNAME s = *this;
		s.toLower();
		return s;
	}

	bool IsNumeric()
	{
		if( GetLength() < 1 )
			return false;
		for( int i=0 ; i<GetLength() ; i++ )
		{
			CHARTYPE c = at(i);
			if( (c < '0') || (c > '9') )
				return false;
		}
		return true;
	}

	ARRAYNAME split(const CLASSNAME& sep) const;
	ARRAYNAME splitExact(const CLASSNAME& sep) const;
	CLASSNAME replace(const CHARTYPE* pOld, const CHARTYPE* pNew) const;
};

/******************************************************************************/
typedef Map<CLASSNAME, CLASSNAME>		MAPNAME;
