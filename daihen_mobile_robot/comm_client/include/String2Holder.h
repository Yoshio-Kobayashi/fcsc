#ifndef __String2Holder_H__
#define __String2Holder_H__

class String2Holder
{
protected:
	Mutex		m_Lock;
	String2		m_sData;

protected:
	String2 get() const
	{
		String2 rc;
		{
			ScopeLock lock(&m_Lock);
			rc = m_sData;
		}
		return rc;
	}

	void set(const WCHAR* p)
	{
		ScopeLock lock(&m_Lock);
		m_sData = p;
	}

public:
	String2Holder()
	{
		set(L"");
	}

	String2Holder(const WCHAR* p)
	{
		set(p);
	}

	virtual ~String2Holder()
	{
	}

	String2Holder& operator=(const WCHAR* p)
	{
		set(p);
		return *this;
	}

	operator String2() const
	{
		return get();
	}
};


#endif
