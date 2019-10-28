#ifndef __Queue_H
#define  __Queue_H

class Queue
{
protected:
	int		m_nRead;	// read position
	int		m_nWrite;	// write position
	int		m_nDatas;	// readable data count
	int		m_nBytes;	// page size of buffer
	int		m_nCount;	// page count of buffer
	int		m_nInts;	// size for calc ptr
	int*	m_pBuffer;	// page size * count
	Mutex	m_Lock;

	//
	int		m_nPeakLength;
	int		m_nPeakDatas;
public:
	Queue();
	virtual ~Queue();

	// basic api
	bool	Create(int nBytes, int nCount);
	void	Free();
	bool	Reset();
	bool	Push(const void* pData, int nLength);
	int		Pop(void* pData);

	// low-level pop
	int		PeekStart(void** ppData);
	void	PeekEnd();

	// performance counter api
	int		GetPeakLength() const;
	int		GetPeakFlow() const;
};

#endif
