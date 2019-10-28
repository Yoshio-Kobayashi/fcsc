
//#include "../d_inc/NetIf_Client.h"
//#include "DaihenRobot.h"
#include "DCommunication/DCommunicationUtil.h"
#include "NetIf.h"

bool NetIf::IsOpen(void)const
{
	return m_Socket.IsOpen();
}

BYTE NetIf::CalcSum(BYTE sum, const BYTE* pData, int nLength)const
{
	for( int i=1 ; i<nLength-1 ; i++ )
	{
		sum += pData[i];
	}
	return sum;
	//#include <openssl/e_os.h>
}

/**
  * リクエスト送信
  * @param cmd コマンド番号
  * @param subcmd サブ・コマンド番号
  * @param req 送信データ
  * @return 処理結果
  */
NetIf::Result NetIf::Send(const CommandCode& cmd, const ByteArray& req)
{
	Socket& sock = m_Socket;
	unsigned short unLen = (unsigned int)req.GetSize();
	unsigned short unAllLen = unLen + 3;

	// send request
	ByteArray sendPacket;
	sendPacket.Add(ENQ);
	sendPacket.Add((unAllLen >> 24) & 0xff);
	sendPacket.Add((unAllLen >> 16) & 0xff);
	sendPacket.Add((unAllLen >>  8) & 0xff);
	sendPacket.Add((unAllLen      ) & 0xff);
	sendPacket.Add(cmd.GetSeqNo());

	sendPacket.Add(m_unDirection | cmd.GetCode());
	sendPacket.Add(cmd.GetSubCode());
	sendPacket.Append(req);

	if( !sock.Send(sendPacket.GetData(), sendPacket.GetSize()) )
	{
		return RESULT_ERROR_SendRequest;
	}

	return RESULT_OK;
}

/**
* 受信
* レスポンスもしくはイベントの受信を行う。
* @param pCmd コマンド番号
* @param pSubcmd サブ・コマンド番号
* @param pData 送信データ
* @return 処理結果
*/
NetIf::Result NetIf::Receive(CommandCode* pCmd, ByteArray* pData)
{
	Socket& sock = m_Socket;
	BYTE recvEnq[5];

	// wait ENQ
	sock.SetReceiveTimeout(1);
	if( !sock.Receive(recvEnq, 5) )
	{
		sock.SetReceiveTimeout(0);
		return RESULT_ERROR_ReceiveEnq;
	}
		sock.SetReceiveTimeout(0);

	if( recvEnq[0] != ENQ )
	{
		return RESULT_ERROR_NotEnq;
	}

	unsigned int unLen =(unsigned int)
		 ((((unsigned int)recvEnq[4]      ) & 0x000000ff)
		| (((unsigned int)recvEnq[3] <<  8) & 0x0000ff00)
		| (((unsigned int)recvEnq[2] << 16) & 0x00ff0000)
		| (((unsigned int)recvEnq[1] << 24) & 0xff000000));


	pData->SetSize((int)unLen);

	if( !sock.Receive(pData->GetData(), unLen) )
	{
		return RESULT_ERROR_ReceiveResponse;
	}

	*pCmd = CommandCode
		(	(CommandCode::Code)((*pData)[1] & 0x7f), // コマンド
			(*pData)[2], // サブコマンド
			(*pData)[0]  // Seq No
		);
	// succeeded
	return RESULT_OK;
}

/**
 * ロボットへの接続
 * @param pIpAddress IPアドレス
 * @return true.成功　false.失敗
 */
bool NetIfClient::Connect(const char* pIpAddress, uint unPort)
{
	if( !m_Socket.Create() )
	{
		return false;
	}
	if( !m_Socket.Connect(pIpAddress, unPort) )
	{
		return false;
	}else
	{
		return true;
	}
}

/**
 * ロボットへの接続解除
 */
void NetIfClient::Close(void)
{
	if(m_Socket.IsOpen())
	{
		m_Socket.Close();
	}
}
