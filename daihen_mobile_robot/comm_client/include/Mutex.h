#ifndef __Mutex_H
#define __Mutex_H

class Mutex
{
protected:
#ifdef _WINDOWS
	CRITICAL_SECTION	m_Native;
#else
	pthread_mutex_t		m_Native;
#endif
public:
	Mutex();
	virtual ~Mutex();
	void Lock();
	void Unlock();
};

#endif
