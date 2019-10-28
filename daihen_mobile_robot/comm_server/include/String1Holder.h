#ifndef __String1Holder_H__
#define __String1Holder_H__

class String1Holder
{
protected:
	Mutex		m_Lock;
	String1		m_sData;

protected:
	String1 get() const
	{
		String1 rc;
		{
			ScopeLock lock(&m_Lock);
			rc = m_sData;
		}
		return rc;
	}

	void set(const char* p)
	{
		ScopeLock lock(&m_Lock);
		m_sData = p;
	}

public:
	String1Holder()
	{
		set("");
	}

	String1Holder(const char* p)
	{
		set(p);
	}

	virtual ~String1Holder()
	{
	}

	String1Holder& operator=(const char* p)
	{
		set(p);
		return *this;
	}

	operator String1() const
	{
		return get();
	}
};


#endif
