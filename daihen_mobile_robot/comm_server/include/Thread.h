#ifndef __Thread_H__
#define __Thread_H__

typedef void* (*PFN_THREAD)(void* pArg);

class Thread
{
public:
#ifdef _WINDOWS
	CWinThread*	m_pNative;
#else
	pthread_t	m_Native;
#endif
	PFN_THREAD	m_pfnEntry;
	void*		m_pArg;
	void*		m_pResult;
public:
	Thread();
	virtual ~Thread();
public:
	bool	Start(PFN_THREAD pfnEntry, void* pAttr, void* pArg, int nPriority);
	void	Join();
};


#endif
