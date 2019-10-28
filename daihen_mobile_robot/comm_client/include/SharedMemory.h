#ifndef __SharedMemory_H
#define __SharedMemory_H

class SharedMemory
{
protected:
	void*		m_pMem;
#ifdef _WINDOWS
	HANDLE		m_hFile;
#endif
public:
	SharedMemory();
	virtual ~SharedMemory();
public:
	void*	Open(const TCHAR* pName, int nProjId, int nSize, bool bReadOnly);
	void	Close();
};

#endif
