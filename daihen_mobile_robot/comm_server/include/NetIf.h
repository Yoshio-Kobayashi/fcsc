/******************************************************************************/
/**
 * @file NetIf.h
 * @brief ネットワークI/F ライブラリ
 * 
 * @author Y.Kajikami
 * @par 更新履歴
 * - 2015/10/06 Kajikami
 *  -# 新規作成
 */

/******************************************************************************/
#ifndef SYSLSI_NETIF_PACKET
#define SYSLSI_NETIF_PACKET
#include "libcpp.h"


class CommandCode;
/**
 * ネットワークI/F パケット
 */
union NetIf_PACKET
{
	struct
	{
		BYTE seq;		//!< Seq No
		BYTE dir;		//!< 方向ビット番号
		BYTE cmd;		//!< コマンド番号
		BYTE subcmd;	//!< サブ・コマンド番号
		unsigned int	 len;		//!< データ長
	};

	BYTE raw[8];		//!< パケットヘッダ
};

union NetIf_Enq
{
	struct{
		BYTE enq;
		unsigned int len;
	};
	BYTE raw[5];
};

unsigned const int s_nPacketHeader = 9;
unsigned const int s_nPacketFooter = 1;

/******************************************************************************/
/**
 * ネットワークI/F クラス
 */
class NetIf
{
public:

	NetIf(unsigned char unDirection)
	:m_unDirection(unDirection), m_Socket(){ }

	NetIf(unsigned char unDirection, Socket socket)
	:m_unDirection(unDirection), m_Socket(socket){ }

	NetIf(const NetIf& arg)
	:m_unDirection(arg.m_unDirection), m_Socket(arg.m_Socket){ }


	NetIf operator=(const NetIf& arg)
	{
		m_unDirection = arg.m_unDirection;
		m_Socket = arg.m_Socket;
		return *this;
	}


	~NetIf(void)
	{
		m_Socket.Close();
	}

	/**
	 * get connection port number
	 * @return port number
	 */
	static int GetPort()
	{
		return 1234;
	}

	/**
	 * @enum marker
	 * 制御信号
	 */
	enum marker
	{
		ACK = 0x06,		//!< ACK信号
		NAK = 0x15,		//!< NAK信号
		ENQ = 0x05		//!< ENQ信号
	};

	/**
	 * @enum Result
	 * 処理結果
	 */
	enum Result
	{
		RESULT_ERROR_SendEnq = 1,				//!< エラー：ENQ送信エラー
		RESULT_ERROR_ReceiveStatus = 2,			//!< エラー：ステータス受信エラー
		RESULT_ERROR_ReceiveNak = 3,			//!< エラー：NAK受信
		RESULT_ERROR_SendRequest = 4,			//!< エラー：リクエスト送信エラー
		RESULT_ERROR_ReceiveEnq = 5,			//!< エラー：ENQ受信エラー
		RESULT_ERROR_NotEnq = 6,				//!< エラー：ENQ受信エラー
		RESULT_ERROR_ReceiveResponse = 7,		//!< エラー：レスポンス受信エラー
		RESULT_ERROR_ReceiveRequest = 8,		//!< エラー：リクエスト受信エラー
		RESULT_ERROR_SendResponse = 9,			//!< エラー：レスポンス送信エラー
		RESULT_ERROR_SendAck = 10,				//!< エラー：ACK送信エラー
		RESULT_ERROR_CheckSum = 11,				//!< エラー：チェックサム不一致
		RESULT_OK = 0							//!< 成功
	};


	/**
	 * @enum Command
	 * コマンド定義
	 */
	enum Command
	{
		CMD_Echo = 0x01		//!< エコーバック
	};

	/**
	 * サムチェックを計算
	 * @param sum 初期値
	 * @param pData データ
	 * @param nLength データ長
	 * @return サムチェック
	 */
	bool IsOpen(void)const;
	BYTE CalcSum(BYTE sum, const BYTE* pData, int nLength)const;
	Result Send(const CommandCode& cmd, const ByteArray& req);
	Result Receive(CommandCode* pCmd, ByteArray* pData);

protected:
	unsigned char 	m_unDirection;
	Socket			m_Socket;
};


class NetIfClient : public NetIf
{
public:
	/**
	 * コンストラクタ
	 * インスタンスの初期化を行う
	 */
	NetIfClient(void)
	:NetIf(0x80)
	{ }

	NetIfClient(const NetIfClient& arg)
	:NetIf(arg)
	{}
	bool Connect(const char* pServerIpAddress, uint unServerPort);
	void Close();
};

class NetIfServer : public NetIf
{
public:
	NetIfServer(void)
	:NetIf(0x00)
	{
	}

	NetIfServer(const NetIfServer& arg)
	:NetIf(arg)
	{}
	/**
	 * コンストラクタ
	 * インスタンスの初期化を行う
	 */
	NetIfServer(const SOCKET& socket)
	: NetIf(0x00)
	{
//		m_Socket.m_hSocket = socket;
#if 0
		char buffer[1024];
		int numrecv = 0;
		while(1)
		{
			numrecv = recv(m_Socket.m_hSocket, buffer, 1024*sizeof(char), 0);
			for(int i = 0; i < numrecv; i++)
			{
				printf("%d[%x]",i, buffer[i]);
			}
			printf("\n");
		}
#endif
	}

	NetIfServer operator=(const NetIfServer& arg)
	{
		m_unDirection = arg.m_unDirection;
		m_Socket = arg.m_Socket;
		return *this;
	}


	void	SetReceiveTimeout(int nRecvTimeout)
	{
		m_Socket.SetReceiveTimeout(nRecvTimeout);
	}

	bool Receive(unsigned char& data)
	{
		return m_Socket.Receive(data);
	}

	bool Receive(void* pData, int nLength)
	{
		return m_Socket.Receive(pData, nLength);
	}

	void Test(void);



	SOCKET GetSOCKET(void)
	{
		return m_Socket.m_hSocket;
	}
};

class NetIfListener : public NetIf
{
public:
	NetIfListener(void)
	:NetIf(0x00), m_DestSOCKET(-1), m_DestSocket()
	{
	}
	bool Listen(int nPort, int nBackLog);
	bool Accept(void);
	Result Receive(CommandCode* pCmd, ByteArray* pData);
	Result Send(const CommandCode& cmd, const ByteArray& req);
#if 0
	bool Recv2(void)
	{
		char buffer[1024];
		int numrecv = 0;
		while(1)
		{
			numrecv = recv(m_DestSocket.m_hSocket, buffer, 1024*sizeof(char), 0);
			for(int i = 0; i < numrecv; i++)
			{
				printf("%d[%x]",i, buffer[i]);
			}
			printf("\n");
		}
		return true;
	}
#else
	bool Recv2(void)
	{
		unsigned char data[5];
		m_DestSocket.SetReceiveTimeout(1);
		if(m_DestSocket.Receive(data,5))
		{
			for(int i = 0; i < 8; i++)
			{
				printf(" %x ", data[i]);
			}
			printf("\n");
			return true;
		}
		return false;
	}
#endif
	SOCKET m_DestSOCKET;
	Socket m_DestSocket;
};
#endif
