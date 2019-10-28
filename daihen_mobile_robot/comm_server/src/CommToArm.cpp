//!
//! @author M.Kubo
//! @par 更新履歴
//! - 2018/06/18 M.Kubo : 新規作成
//! - 2018/06/20 M.Kubo : 移動の不具合修正
//!

#include "CommToArm.h"


#define PRINT_LOG
#ifdef PRINT_LOG
	#define MD_PRINTF(...)	printf(__VA_ARGS__)
#else
	#define MD_PRINTF(...)
#endif

//! ****************************************************************************
//! コンストラクタ
//! @param[in] unPort 通信に使用するポート番号
CommToArm::CommToArm(void)
	: m_NetIfServer()
	, m_pos_x(0)
	, m_pos_y(0)
	, m_rad(0)
	, m_pos_z(0)
	, m_nCurrentPush(0)
	, m_nCurrentPop(0)
{}

//! ****************************************************************************
//! デストラクタ
CommToArm::~CommToArm(void)
{}

void CommToArm::Execute()
{

	static CommandCode recvCommand;
	static ByteArray recvData(5);

	NetIf::Result result;

	result = m_NetIfServer.Receive(&recvCommand, &recvData);

	if(NetIf::RESULT_OK == result)
	{
		HandleCommand(recvCommand, recvData);
	}
	else if(HasSendingCommand())
	{
		SendNextCommand();
	}

}

void CommToArm::HandleCommand(const CommandCode& recvCommand, const ByteArray& recvData) {
	MD_PRINTF("MD recv cmd:%x, sub:%x\n"
					, recvCommand.GetCode(), recvCommand.GetSubCode());

	switch (recvCommand.GetCode()) {
	case CommandCode::MASTER_ORDER: {
		//アクション指令
		HandleOrderCommand(recvCommand,recvData);
		break;
	}
	case CommandCode::MASTER_COM: {
		HandleMasterDeviceCommand(recvCommand, recvData);
		break;
	}
	default: {
		break;
	}
	}

}

//! ****************************************************************************
//! 共有メモリにマスター装置から受信した情報を書き込み、CommunicationLoopクラスへ渡す
//! @param[in] command コマンド
//! @param[in] data 受信データ
//! @retval 異常
CommToArm::DError CommToArm::HandleOrderCommand
		(const CommandCode& command,const ByteArray& data)
{
	switch(command.GetSubCode())
	{
		// 直値移動アクションとマスター切替アクションのみ実装
		case CommandCode::SUB_ACTION:
		{
			float r_x;
			float r_y;
			float r_rad;
			float r_z;

			//アクション受付応答送信
			AddSendingCommand(command);
			unsigned int unReadNum = 5; //1から4のデータは無視(ロボット番号など)

			//! [byte] 座標系種別
			ComUtil::GetChar(&unReadNum, data, unReadNum);
			//! [byte] 目標位置種別
			ComUtil::GetChar(&unReadNum, data, unReadNum);

			//! [byte]     目標位置姿勢・地図ID・番号
			//! [char[20]] 目標位置姿勢・地図ID・名前
			unReadNum += 21;

			//! [float[4]] 目標位置姿勢
			//ダミー座標移動
			r_x 	= ComUtil::GetFloat(&unReadNum, data, unReadNum);					// X座標
			r_y 	= ComUtil::GetFloat(&unReadNum, data, unReadNum); 					// Y座標
			r_rad   = ComUtil::GetFloat(&unReadNum, data, unReadNum);				// 姿勢[rad]
			r_z 	= ComUtil::GetFloat(&unReadNum, data, unReadNum);

			m_pos_x = m_pos_x + cos(m_rad) * r_x - sin(m_rad) * r_y ;
			m_pos_y = m_pos_y + sin(m_rad) * r_x + cos(m_rad) * r_y ;
			m_rad   = m_rad   + r_rad ;
			m_pos_z = m_pos_z + r_z ;

			//radの範囲修正
			while (m_rad <= -M_PI)
			{
				m_rad += 2 * M_PI;
			}
			while (M_PI < m_rad)
			{
				m_rad -= 2 * M_PI;
			}

			//アクション完了応答送信
			AddSendingCommand(CommandCode(CommandCode::MASTER_COM, CommandCode::SUB_MASTER_SUB_ACTION_FINISHED));
			break;
		}

		default:
		{
			printf("HandleOrderCommand unknown command %d\n", command.GetSubCode());
		}
	}
	return NOT_ERROR;
}

//! ****************************************************************************
//! マスター装置から通信コマンドを受信する。
//! @param[in] command コマンド
//! @param[in] data 受信データ
//! @retval 受信時に発生した異常
CommToArm::DError CommToArm::HandleMasterDeviceCommand
	(const CommandCode& command,const ByteArray& data)
{

	switch(command.GetSubCode())
	{
		case CommandCode::SUB_MASTER_REQ_STATE:
		{
			break;
		}
		case CommandCode::SUB_MASTER_REQ_POS:
		{
			//座標情報通知コマンド生成
			AddSendingCommand(command);
			break ;
		}
		case CommandCode::SUB_MASTER_ERROR_NOTIFICATION:
		{

			break;
		}

		default:
		{
			MD_PRINTF("Receive InvalidCommand(%d, %d)\n"
					, command.GetCode(),  command.GetSubCode());
			return ERROR;
		}
	}

	return NOT_ERROR;
}

//! ****************************************************************************
//! キューに存在する通信コマンドをマスター装置に送信する。

CommToArm::DError CommToArm::SendNextCommand() {

	DError SendingError;
	ByteArray data;

	CommandCode sendCommand = GetCommand();

	SendingError = NOT_ERROR;
	MD_PRINTF("MD send cmd:%x, sub:%x\n", sendCommand.GetCode(), sendCommand.GetSubCode());

	switch (sendCommand.GetCode()) {
		case CommandCode::MASTER_ORDER: {
			SendingError = SendOrderCommand (sendCommand);
			break;
		}
		case CommandCode::MASTER_COM: {
			switch (sendCommand.GetSubCode())
			{
				case CommandCode::SUB_MASTER_REQ_STATE :
				{
					break ;
				}
				case CommandCode::SUB_MASTER_REQ_POS :
				{
					ComUtil::AddFloatToUcVector(&data, m_pos_x);
					ComUtil::AddFloatToUcVector(&data, m_pos_y);
					ComUtil::AddFloatToUcVector(&data, m_rad);
					ComUtil::AddFloatToUcVector(&data, m_pos_z);
					if(m_NetIfServer.Send(sendCommand,data))
					{
						SendingError = ERROR;
					}
					break ;
				}
				case CommandCode::SUB_MASTER_ERROR_NOTIFICATION :
				{
					break;
				}
				case CommandCode::SUB_MASTER_SUB_ACTION_FINISHED :
				{
					ByteArray data;
					data.Add(1); // ロボット
					data.Add(0x00); // 結果
				//	float f[4];

					ComUtil::AddIntToUcVector(&data, 1);
					for(int i = 0; i < 20 ; i++)
					{
						data.Add(' ');
					}

					ComUtil::AddFloatToUcVector(&data, m_pos_x);
					ComUtil::AddFloatToUcVector(&data, m_pos_y);
					ComUtil::AddFloatToUcVector(&data, m_rad);
					ComUtil::AddFloatToUcVector(&data, m_pos_z);
					printf("send action finished\n");
					if(m_NetIfServer.Send(sendCommand, data))
					{
						SendingError = ERROR;
					}

					break;
				}
				default: {
					break;
				}
			}
			break;
		}
		default: {
			break;
		}
	}

	return SendingError;

}


//!*****************************************************************************
//!
CommToArm::DError CommToArm::SendOrderCommand (const CommandCode& command)
{
	ByteArray data;

	switch(command.GetSubCode())
	{
		case CommandCode::SUB_ACTION:
		{
			data.Add(0); //!< ロボット番号(未使用）
			data.Add(1); //!< アクション番号（未使用）
			data.Add(0);

			break;
		}
		default:
		{
			printf("SendOrderCommand unknown command %d\n",  command.GetSubCode());
		}
	}


	if(m_NetIfServer.Send(command, data))
	{
		return ERROR;
	}

	return NOT_ERROR;
}

//! ****************************************************************************
//! 送信すべきコマンドがあることを確認する。
//! @retval 送信すべきコマンドがある場合true
bool CommToArm::HasSendingCommand(void)
{
	return (m_nCurrentPush != m_nCurrentPop);
}

//! ****************************************************************************
//! 送信するコマンドを追加する。
//! @param[in] code 追加するコマンドコード
void CommToArm::AddSendingCommand(CommandCode code)
{
	int nNextPush = (m_nCurrentPush + 1) % nMaxQueue;
	if(nNextPush != m_nCurrentPop)
	{
		commandQueue[m_nCurrentPush] = code;
		m_nCurrentPush = nNextPush;
	}else
	{
		printf("ERROR Queue Is Full\n");
	}
}
//! ****************************************************************************
//! キューからコマンドを抜き出す。
//! @retval 抜き出されたコマンドコード
CommandCode CommToArm::GetCommand(void)
{
	if(m_nCurrentPush != m_nCurrentPop)
	{
		int nCurrentPop = m_nCurrentPop;
		m_nCurrentPop = (m_nCurrentPop + 1) % nMaxQueue;
		return commandQueue[nCurrentPop];
	}else
	{
		printf("ERROR Queue Is Empty\n");
		return CommandCode(CommandCode::INITIALIZE
						, CommandCode::SUB_INITIALIZE);
	}
}

