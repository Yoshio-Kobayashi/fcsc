class ScopeLock
{
protected:
	Mutex*	m_pMutex;
public:
	ScopeLock(const Mutex* pMutex)
	{
		m_pMutex = (Mutex*)pMutex;
		m_pMutex->Lock();
	}
	virtual ~ScopeLock()
	{
		m_pMutex->Unlock();
	}
};
