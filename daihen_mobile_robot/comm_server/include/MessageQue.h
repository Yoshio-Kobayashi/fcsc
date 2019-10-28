#ifndef __MessageQue_H
#define __MessageQue_H

class MessageQue
{
protected:
	HANDLE	m_nId;
	bool	m_bConnected;
public:
	MessageQue();
	virtual ~MessageQue();
public:
	bool	Open(const TCHAR* pName, int nProjId, bool bOwner=false);
	HANDLE	GetHandle() const;
	bool	IsOpened() const;
	void	Close();
	bool	Destroy();
	bool	Send(const TCHAR* pData);
	String	Recv();
};

#endif
