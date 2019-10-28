#ifndef _WINDOWS
#define INVALID_SOCKET			(-1)
typedef int						SOCKET;
#endif

class Socket
{
protected:

	void*		m_pSSL;
	String1		m_sLastError;
	bool		m_bOpened;
	DWORD		m_dwConnectTimeout;
	int			m_nRecvTimeout;
	bool		m_bConnected;
public:
	SOCKET		m_hSocket;
	int			m_nDebugStatus;
	int			m_nLogId;
public:
	static bool Initialize();

	Socket();
	Socket(const Socket& arg);
	virtual ~Socket();
	Socket operator = (const Socket& arg);

	virtual bool IsOpen() const;
	virtual bool IsConnected() const;
	virtual bool Create();
	virtual bool Accept(SOCKET* pSocket);
	virtual bool Listen(int nPort, int nBackLog=5);
	virtual bool Connect(const TCHAR* pHost, int nPort);
	virtual bool NegotiateSsl();
	virtual bool Receive(unsigned char& data);
	virtual bool Receive(void* pData, int nLength);
	virtual bool Send(const void* pData, int nLength);
	virtual void Close();

	void	SetReceiveTimeout(int nRecvTimeout)
	{ m_nRecvTimeout = nRecvTimeout;}

	virtual bool SendLine(const char* pLine);
#ifdef _UNICODE
	virtual bool SendLine(const WCHAR* pLine);
#endif
	virtual bool ReceiveLine(String1& sResult);
#ifdef _UNICODE
	virtual bool ReceiveLine(String2& sResult);
#endif

	operator SOCKET()
	{
		return m_hSocket;
	}
};
