/******************************************************************************/
#include <libcpp.h>

/******************************************************************************/
#if defined(LIBCPP_SSL_DYNAMIC) || defined(LIBCPP_SSL_STATIC)
#define USE_SSL
#include <openssl/crypto.h>	// �Í��֌W
#include <openssl/x509.h>	// �ؖ����֌W
#include <openssl/pem.h>	// �ؖ����֌W
#include <openssl/ssl.h>	// SSL�֌W
#include <openssl/err.h>	// �G���[�����֘A
//#include <openssl/e_os.h>
//#include <openssl/bio.h>

#pragma comment(lib, "libeay32.lib")
#pragma comment(lib, "ssleay32.lib")
void*	s_pSSLContext = NULL;
#endif

#ifdef _WINDOWS
typedef int		socklen_t;
#else
#define	closesocket				close
typedef struct sockaddr_in		SOCKADDR_IN;
#endif

/******************************************************************************/
bool InitSocket()
{
	static bool s_bInitialized = false;
	if( s_bInitialized )
		return true;

#ifdef _WINDOWS
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 0), &wsaData);
#endif
	s_bInitialized = true;
	return true;
}

/******************************************************************************/
static int connect2(SOCKET soc, struct sockaddr* pAddr, int nAddrLen, DWORD dwTimeout)
{
#ifdef _WINDOWS
	// �C�x���g�쐬
	WSAEVENT hEvent = WSACreateEvent();
	if( hEvent == WSA_INVALID_EVENT )
		return -1;

	// �C�x���g�^�Ɂi�����I�Ƀm���u���b�L���O�ɂȂ�j
	if( WSAEventSelect(soc, hEvent, FD_CONNECT) == SOCKET_ERROR )
	{
		WSACloseEvent(hEvent);
		return -2;
	}

	try
	{
		// �ڑ�
		if( connect(soc, pAddr, sizeof(*pAddr)) == SOCKET_ERROR )
		{
			if( WSAGetLastError() != WSAEWOULDBLOCK )
				throw -3;
		}

		//�C�x���g�����ҋ@
		DWORD rc = WSAWaitForMultipleEvents(1, &hEvent, FALSE, dwTimeout, FALSE);
		if( rc == WSA_WAIT_TIMEOUT )
			throw -100;
		if( rc != WSA_WAIT_EVENT_0 )
			throw -4;

		WSANETWORKEVENTS events;
		if( WSAEnumNetworkEvents(soc, hEvent, &events) == SOCKET_ERROR )
			throw -5;
		if( !(events.lNetworkEvents & FD_CONNECT) )
			throw -6;
		if( events.iErrorCode[FD_CONNECT_BIT] )
			throw -7;

		throw 0;
	}
	catch(int rc)
	{
		WSAEventSelect(soc, NULL, 0);

		DWORD dw = 0;
		ioctlsocket(soc, FIONBIO, &dw);

		WSACloseEvent(hEvent);
		return rc;
	}
#else
	
	return connect(soc, (struct sockaddr*)pAddr, sizeof(*pAddr));
#endif
}

/******************************************************************************/
bool Socket::Initialize()
{
	if( !InitSocket() )
		return false;

	//サーバ断（BROKEN SIGPIPE)によるシステムダウンを防ぐ
	//SIGPIPEを無視(SIG_IGN)するように設定。
	//シミュレータではシステムコールの変更ができないので、
	//別の案(sendにSIG_NOSIGNALを付加)を採用。
	//struct sigaction action, oldaction;
	//action.sa_handler = SIG_IGN;
	//sigemptyset(&action.sa_mask);
	//action.sa_flags = SA_RESTART; //シグナルに割り込まれたシステムコールを再起動
	//SIGPIPEにactionを設定し、oldactionに元のアクションが格納される。
	//sigaction(SIGPIPE, &action, &oldaction);*/

	#ifdef USE_SSL
	if( s_pSSLContext )
		return true;

	SSLeay_add_ssl_algorithms();
	SSL_load_error_strings();
	SSL_library_init();
	s_pSSLContext = SSL_CTX_new(SSLv2_client_method());
	return (s_pSSLContext != NULL);
#else
	return true;
#endif
}

/******************************************************************************/
Socket::Socket()
{
	m_hSocket = INVALID_SOCKET;
	m_pSSL = NULL;
	m_dwConnectTimeout = 10 * 1000;
	m_nRecvTimeout = 0;
	m_nLogId = 0;
	m_bConnected = false;

	Initialize();
}
Socket::Socket(const Socket& arg)
: m_pSSL(arg.m_pSSL)
, m_sLastError(arg.m_sLastError)
, m_bOpened(arg.m_bOpened)
, m_dwConnectTimeout(arg.m_dwConnectTimeout)
, m_nRecvTimeout(arg.m_nRecvTimeout)
, m_bConnected(arg.m_bConnected)
, m_hSocket(arg.m_hSocket)
, m_nDebugStatus(arg.m_nDebugStatus)
, m_nLogId(arg.m_nLogId)
{;}

Socket Socket::operator = (const Socket& arg)
{
	m_pSSL = (arg.m_pSSL);
	m_sLastError = (arg.m_sLastError);
	m_bOpened = (arg.m_bOpened);
	m_dwConnectTimeout = (arg.m_dwConnectTimeout);
	m_nRecvTimeout = (arg.m_nRecvTimeout);
	m_bConnected = (arg.m_bConnected);
	m_hSocket = (arg.m_hSocket);
	m_nDebugStatus = (arg.m_nDebugStatus);
	m_nLogId = (arg.m_nLogId);
	return *this;
}
/******************************************************************************/
Socket::~Socket()
{
	Close();
}

/******************************************************************************/
bool Socket::IsOpen() const
{
	return (m_hSocket != INVALID_SOCKET);
}

/******************************************************************************/
bool Socket::IsConnected() const
{
	return m_bConnected;
}

/******************************************************************************/
bool Socket::Create()
{
	Close();
	m_hSocket = socket(AF_INET, SOCK_STREAM, 0);
	return IsOpen();
}

/******************************************************************************/
bool Socket::Connect(const TCHAR* pHost, int nPort)
{
	m_nDebugStatus = 100;
	String1 sHost = T2M(pHost);

	struct sockaddr_in sa;
	sa.sin_family = AF_INET;
	sa.sin_port = htons(nPort);
	sa.sin_addr.s_addr = inet_addr(sHost);

	if( sa.sin_addr.s_addr == ((DWORD)-1) )
	{
		struct hostent* host = gethostbyname(sHost);
		if( host )
		{
			for( unsigned int **addrptr=(unsigned int **)host->h_addr_list ; *addrptr ; addrptr++ )
			{
				sa.sin_addr.s_addr = *(*addrptr);
				m_nDebugStatus = 101;
				if( connect2(m_hSocket, (struct sockaddr*)&sa, sizeof(sa), m_dwConnectTimeout) == 0 )
				{
					m_nDebugStatus = 109;
					m_bConnected = true;
					return true;
				}
				m_nDebugStatus = 102;
			}
		}
	}
	else
	{
		m_nDebugStatus = 101;
		if( connect2(m_hSocket, (struct sockaddr*)&sa, sizeof(sa), m_dwConnectTimeout) == 0 )
		{
if( m_nLogId )
{
	_unlink(Edit1("C:\\test\\recv_%d.txt", m_nLogId));
}
			m_nDebugStatus = 109;
			m_bConnected = true;
			return true;
		}
		m_nDebugStatus = 102;
	}
	Close();
	m_nDebugStatus = 103;
	return false;
}

/******************************************************************************/
bool Socket::Listen(int nPort, int nBackLog)
{
	SOCKADDR_IN sa;
	sa.sin_family = AF_INET;
#ifdef _WINDOWS
	sa.sin_addr.S_un.S_addr = INADDR_ANY;
#else
	sa.sin_addr.s_addr = INADDR_ANY;
#endif
	sa.sin_port = htons(nPort);
	if( bind(m_hSocket, (struct sockaddr*)&sa, sizeof(sa)) != 0 )
		return false;

	if( listen(m_hSocket, nBackLog) != 0 )
		return false;

	return true;
}

/******************************************************************************/
bool Socket::Accept(SOCKET* pSocket)
{
	SOCKADDR_IN sa;
	memset(&sa, 0x00, sizeof(sa));
	socklen_t nAddrLen = sizeof(sa);

	SOCKET s = accept(m_hSocket, (struct sockaddr*)&sa, &nAddrLen);
	*pSocket = s;
#if 0
// 通信成功
		char buffer[1024];
		int numrecv = 0;
		while(1)
		{
			numrecv = recv(*pSocket, buffer, 1024*sizeof(char), 0);
			for(int i = 0; i < numrecv; i++)
			{
				printf("%x", buffer[i]);
			}
			printf("\n");
		}
#endif
	return (s != INVALID_SOCKET);
}

/******************************************************************************/
void Socket::Close()
{
#ifdef USE_SSL
	// shutdown
	if( m_pSSL )
		SSL_shutdown((SSL*)m_pSSL);

	// free
	//if( m_pPeerCert )
	//{
	//	X509_free((X509*)m_pPeerCert);
	//	m_pPeerCert = NULL;
	//}
	if( m_pSSL )
	{
		SSL_free((SSL*)m_pSSL);
		m_pSSL = NULL;
	}
#endif

	if( m_hSocket != INVALID_SOCKET )
	{
		closesocket(m_hSocket);
		m_hSocket = INVALID_SOCKET;
	}
	m_bConnected = false;
}

/******************************************************************************/
bool Socket::NegotiateSsl()
{
#ifdef USE_SSL
	try
	{
		if( m_pSSL )
			throw "already negotiated.";

		m_pSSL = SSL_new((SSL_CTX*)s_pSSLContext);
		if( !m_pSSL )
			throw "SSL_new() failed.";

		int rc;
		rc = SSL_set_fd((SSL*)m_pSSL, m_hSocket);
		if( rc != 1 )
			throw "SSL_set_fd() failed.";

		rc = SSL_connect((SSL*)m_pSSL);
		if( rc != 1 )
			throw "SSL_connect() failed.";

		return true;
	}
	catch(const char* pError)
	{
		m_sLastError = pError;
		return false;
	}
#else
	m_sLastError = "not support.";
	return false;
#endif
}

/******************************************************************************/
bool Socket::Receive(unsigned char& data)
{
	return Receive(&data, 1);
}

/******************************************************************************/
bool Socket::Receive(void* pData, int nLength)
{
	BYTE* p = (BYTE*)pData;

	int tmpTimeOut = m_nRecvTimeout;
	if( m_nRecvTimeout > 0 )
	{
		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(m_hSocket, &rfds);

		struct timeval to;
		to.tv_sec = m_nRecvTimeout / 1000;
		to.tv_usec = (m_nRecvTimeout % 1000) * 1000;

		int rc = select(m_hSocket+1, &rfds, NULL, NULL, &to);

		if( rc <= 0 )
		{
			return false;
		}		
		m_nRecvTimeout = 0;
	}

	for( int nLeft=nLength ; nLeft>0 ; )
	{
		if( m_nRecvTimeout > 0 )
		{
			fd_set rfds;
			FD_ZERO(&rfds);
			FD_SET(m_hSocket, &rfds);

			struct timeval to;
			to.tv_sec = m_nRecvTimeout / 1000;
			to.tv_usec = (m_nRecvTimeout % 1000) * 1000;

			int rc = select(m_hSocket+1, &rfds, NULL, NULL, &to);
			if( rc == 0 )
			{
				m_sLastError = "receive timeout.";
				return false;
			}
			else if( rc < 0 )
			{
				m_sLastError = "receive failed.";
				return false;
			}
		}

		int n;
#ifdef USE_SSL
		if( m_pSSL )
			n = SSL_read((SSL*)m_pSSL, p, nLeft);
		else
#endif
			n = recv(m_hSocket, (char*)p, nLeft, 0);
//		numrecv = recv(pDestSocket->m_hSocket, buffer, sizeof(char)*1024, 0);
		if( n < 0 )
			Close();
		if( n < 1 )
		{
			m_nRecvTimeout = tmpTimeOut;
			return false;
		}

		if( m_nLogId )
		{
			FILE* fp = fopen(Edit1("C:\\test\\recv_%d.txt", m_nLogId), "ab");
			if( fp )
			{
				fwrite(p, n, 1, fp);
				fclose(fp);
			}
		}

		p += n;
		nLeft -= n;
	}
/*	printf("\nRecv:");
	for(int i = 0; i < nLength; i++)
	{
		printf("[%d:%2x]",i, *((char*)pData+i));
	}
	printf("\n");*/
	m_nRecvTimeout = tmpTimeOut;
	return true;
}

/******************************************************************************/
bool Socket::ReceiveLine(String1& sResult)
{
	sResult = "";

	for( BYTE c ; Receive(c) ; )
	{
		if( c == '\r' )
			continue;
		if( c == '\n' )
			return true;
		sResult += ((char)c);
	}
	return false;
}

/******************************************************************************/
#ifdef _UNICODE
bool Socket::ReceiveLine(String2& sResult)
{
	String1 line;
	if( !ReceiveLine(line) )
		return false;
	sResult = U2W(line);
	return true;
}
#endif

/******************************************************************************/
bool Socket::Send(const void* pData, int nLength)
{
	int nSent;
#ifdef USE_SSL
	if( m_pSSL )
		nSent = SSL_write((SSL*)m_pSSL, pData, nLength);
	else
#endif
/*	printf("\nSend:");
	for(int i = 0; i < nLength; i++)
	{
		printf("[%d:%2x]",i, *((char*)pData+i));
	}
	printf("\n");*/
		//サーバ断（BROKEN SIGPIPE)によるシステムダウンを防ぐ(MSG_NOSIGNAL)
		nSent = send(m_hSocket, (const char*)pData, nLength, MSG_NOSIGNAL);
		if(nSent == -1)
		{
			switch(errno)
			{
			case EPIPE:
			{
				printf("BROKEN_SIGPIPE\n");
				break;
			}
			default:
			{
				break;
			}
			}
		}
	return (nSent == nLength);
}

/******************************************************************************/
bool Socket::SendLine(const char* pData)
{
	String1 buf = pData;
	buf += "\r\n";
	return Send(buf.c_str(), buf.length());
}

/******************************************************************************/
#ifdef _UNICODE
bool Socket::SendLine(const WCHAR* pData)
{
	return SendLine(W2M(pData));
}
#endif
