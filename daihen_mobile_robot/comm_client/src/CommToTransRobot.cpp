/*
 * CommToTransRobot.cpp
 * 搬送ロボットと通信するためのクラス
 *
 *  Created on: 2018/06/08
 *      Author: M.Kubo
 */

#include "CommToTransRobot.h"


CommToTransRobot::CommToTransRobot(void)
: m_orderDevice(CommToTransRobot::SERVER)
, m_masterID(1)
, m_ctrlSig(0)
, m_error(0)
, m_pos_x(0)
, m_pos_y(0)
, m_rad(0)
, m_pos_z(0)
, m_nCurrentPush(0)
, m_nCurrentPop(0)
{
}

bool CommToTransRobot::Connect(const char* pServerIpAddress, uint unServerPort)
{
	return m_NetIfClient.Connect(pServerIpAddress, unServerPort);
}

void CommToTransRobot::Close(void)
{
	m_NetIfClient.Close();
}

void CommToTransRobot::Execute(void)
{
	static CommandCode recvCommand;
	static ByteArray recvData(5);

	NetIf::Result result;

	//ReceiveDataShared共有メモリを確認し、必要なら通信コマンド生成
	//CreateNewCommand();

	result = m_NetIfClient.Receive(&recvCommand, &recvData);

	if(NetIf::RESULT_OK == result)
	{
		RecvCommandProcess(recvCommand, recvData);

	}else if(HasSendingCommand())
	{
		SendNextCommand();
	}

}


CommToTransRobot::DError CommToTransRobot::RecvCommandProcess(const CommandCode& recvCommand, const ByteArray& recvData) {

	CommToTransRobot::DError error;

	printf("MD recv cmd:%x, sub:%x\n", recvCommand.GetCode(), recvCommand.GetSubCode());

	switch (recvCommand.GetCode()) {
		case CommandCode::ORDER: {
			//アクション指令応答受信
			error = RecvOrderCommand(recvCommand,recvData);
			break;
		}
		case CommandCode::MASTER_COM: {
			error = RecvMasterDeviceCommand(recvCommand, recvData);
			break;
		}
		default: {
			break;
		}
	}

	return error;


}


//! ****************************************************************************
//! 共有メモリにマスター装置から受信した情報を書き込み、CommunicationLoopクラスへ渡す
//! @param[in] command コマンド
//! @param[in] data 受信データ
//! @retval 異常
CommToTransRobot::DError CommToTransRobot::RecvOrderCommand
		(const CommandCode& command,const ByteArray& data)
{
	uint unElemNum = 3;
	CommToTransRobot::DError error = NOT_ERROR;

	switch(command.GetSubCode())
	{
		// アクションの応答
		case CommandCode::SUB_ACTION:
		{
			ComUtil::GetChar(&unElemNum, data, unElemNum); //ゴミ
			ComUtil::GetChar(&unElemNum, data, unElemNum); //ゴミ
			error = (DError)ComUtil::GetChar(&unElemNum, data, unElemNum);

			break;
		}

		default:
		{
			printf("HandleOrderCommand unknown command %d\n", command.GetSubCode());
		}
	}
	return error;
}



//! ****************************************************************************
//! マスター装置から通信コマンドを受信する。
//! @param[in] command コマンド
//! @param[in] data 受信データ
//! @retval 受信時に発生した異常
CommToTransRobot::DError CommToTransRobot::RecvMasterDeviceCommand
	(const CommandCode& command,const ByteArray& data)
{
	uint unElemNum = 3;

	switch(command.GetSubCode())
	{
		case CommandCode::SUB_MASTER_REQ_STATE:
		{
			//MASTER_DEVICEのときアームからの動作指令を受け取る
			m_orderDevice = (OrderDevice)ComUtil::GetChar(&unElemNum, data, unElemNum);

			//未使用
			m_masterID = ComUtil::GetChar(&unElemNum, data, unElemNum);
			m_ctrlSig = ComUtil::GetFloat(&unElemNum, data, unElemNum);
			m_error = ComUtil::GetChar(&unElemNum, data, unElemNum);

			printf("recv REQ STATE: ORDER_DEVICE = %d\n", m_orderDevice);

			//応答を返す
			AddSendingCommand(command);
			break;
		}
		case CommandCode::SUB_MASTER_REQ_POS:
		{
			m_pos_x = ComUtil::GetFloat(&unElemNum, data, unElemNum);
			m_pos_y = ComUtil::GetFloat(&unElemNum, data, unElemNum);
			m_rad   = ComUtil::GetFloat(&unElemNum, data, unElemNum);
			m_pos_z = ComUtil::GetFloat(&unElemNum, data, unElemNum);

			break ;
		}
		case CommandCode::SUB_MASTER_ERROR_NOTIFICATION:
		{
			//エラー処理
			break;
		}
		case CommandCode::SUB_MASTER_SUB_ACTION_FINISHED:
		{

			unElemNum = 29;

			m_pos_x = ComUtil::GetFloat(&unElemNum, data, unElemNum);
			m_pos_y = ComUtil::GetFloat(&unElemNum, data, unElemNum);
			m_rad   = ComUtil::GetFloat(&unElemNum, data, unElemNum);
			m_pos_z = ComUtil::GetFloat(&unElemNum, data, unElemNum);
			break;
		}
		default:
		{
			printf("Receive InvalidCommand(%d, %d)\n", command.GetCode(),  command.GetSubCode());
		}
	}

	return NOT_ERROR;
}

//! ****************************************************************************
//! キューに存在する通信コマンドをマスター装置に送信する。

CommToTransRobot::DError CommToTransRobot::SendNextCommand() {

	DError SendingError;
	ByteArray data;

	CommandCode sendCommand = GetCommand();

	printf("MD send cmd:%x, sub:%x\n", sendCommand.GetCode(), sendCommand.GetSubCode());

	SendingError = NOT_ERROR;

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
					ComUtil::AddUcToUcVector(&data, m_orderDevice);
					ComUtil::AddUcToUcVector(&data, m_masterID);
					ComUtil::AddFloatToUcVector(&data, m_ctrlSig);
					ComUtil::AddUcToUcVector(&data, 0x00);    //エラー信号

					if(m_NetIfClient.Send(sendCommand,data))
					{
						SendingError = ERROR;
					}
					break ;
				}
				case CommandCode::SUB_MASTER_REQ_POS :
				{
					if(m_NetIfClient.Send(sendCommand,data))
					{
						SendingError = ERROR;
					}
					break ;
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
CommToTransRobot::DError CommToTransRobot::SendOrderCommand (const CommandCode& command)
{
	ByteArray data;

	switch(command.GetSubCode())
	{
		case CommandCode::SUB_ACTION:
		{
			GenMoveActionData(&data) ;
			break;
		}

		case CommandCode::SUB_MASTER_SWITCH_ACTION:
		{
			//未使用
			break;
		}
		default:
		{
			printf("SendOrderCommand unknown command %d\n",  command.GetSubCode());
			break;
		}
	}

	if(m_NetIfClient.Send(command, data))
	{
		return ERROR;
	}

	return NOT_ERROR;
}


CommToTransRobot::DError CommToTransRobot::SendMoveActionCommand(float x, float y, float rad, float z)
{

	CommandCode recvCommand;
	ByteArray recvData(5);

	ByteArray data(0);

	NetIf::Result result;
	DError error;
	int cycle_num;

	bool b_doneAction = false;

	m_dmove_pos_x = x ;				// X座標
	m_dmove_pos_y = y ; 			// Y座標
	m_dmove_rad   = rad;			// 姿勢[rad]
	m_dmove_pos_z = z ;				// リフタ高


	//直値移動の送信処理
	AddSendingCommand(CommandCode(CommandCode::MASTER_ORDER, CommandCode::SUB_ACTION));

	SendNextCommand();

	//座標応答の受信処理
	result = m_NetIfClient.Receive(&recvCommand, &recvData);
	cycle_num = 1;

	// printf("%02x %02x:", recvCommand.GetCode(), recvCommand.GetSubCode());
	// for (size_t i = 0; i < recvData.size(); i++) {
	// 	printf("%02x", recvData[i]);
	// }
	// printf("\n");

	while (!b_doneAction && cycle_num < RECV_CONFIRM_NUM)
	{
		cycle_num++;
		usleep(RECV_CONFIRM_CYCLE_MS * 1000); //1[s]受信待ち。
		result = m_NetIfClient.Receive(&recvCommand, &recvData);

		// printf("%02x %02x:", recvCommand.GetCode(), recvCommand.GetSubCode());
		// for (size_t i = 0; i < recvData.size(); i++) {
		// 	printf("%02x", recvData[i]);
		// }
		// printf("\n");

		if (result == NetIf::RESULT_OK)
		{
			//アクションの受付完了
			if (recvCommand.GetCode() == CommandCode::MASTER_ORDER && recvCommand.GetSubCode() == CommandCode::SUB_ACTION)
			{
				//特に動作なし
			}

			//アクションの実行完了
			if (recvCommand.GetCode() == CommandCode::MASTER_COM && recvCommand.GetSubCode() == CommandCode::SUB_MASTER_SUB_ACTION_FINISHED)
			{
				b_doneAction = true;
			}
		}
	}

	if (result == NetIf::RESULT_OK) {
		error = NOT_ERROR;
	}
	else
	{
		error = ERROR;
	}

	if (error == ERROR)
	{
		printf("move action recv error\n");
		return error;
	}

	error = RecvCommandProcess(recvCommand, recvData);

	return error ;
}

void CommToTransRobot::GenMoveActionData(ByteArray* data)
{
	ComUtil::AddUcToUcVector(data, (uchar)1);				//! [byte] ロボット番号(未使用)
	ComUtil::AddUcToUcVector(data, (uchar)1);				//! [byte] アクション指令(直値)
	ComUtil::AddUcToUcVector(data, (uchar)1);				//! [byte] 座標系種別：： 0:教示点 1:ロボット座標 2:区画図座標
	ComUtil::AddUcToUcVector(data, (uchar)1);				//! [byte] 目標位置種別:: 1:相対値
	ComUtil::AddUcToUcVector(data, (uchar)1);				//! [byte] 目標位置姿勢・地図ID・番号

	//! [char[20]] 目標位置姿勢・地図ID・名前
	for(int i = 0;i < 20;i++) {
		ComUtil::AddUcToUcVector(data, (uchar)0);
	}

	//ロボット座標に対する相対的移動
	ComUtil::AddFloatToUcVector(data, m_dmove_pos_x);	//! [float[4]] 目標位置姿勢X
	ComUtil::AddFloatToUcVector(data, m_dmove_pos_y);	//! [float[4]] 目標位置姿勢Y
	ComUtil::AddFloatToUcVector(data, m_dmove_rad);	//! [float[4]] 目標位置姿勢rad
	ComUtil::AddFloatToUcVector(data, m_dmove_pos_z);	//! [float[4]] 目標位置姿勢z

	ComUtil::AddFloatToUcVector(data, (float)0.4);	//! [float[4]] 速度.並進速度.位置_X
	ComUtil::AddFloatToUcVector(data, (float)0.4);	//! [float[4]] 速度.並進速度.位置_Y
	ComUtil::AddFloatToUcVector(data, (float)0.5);	//! [float[4]] 速度.回転速度
	ComUtil::AddFloatToUcVector(data, (float)0.01);	//! [float[4]] 速度.リフタ速度

	ComUtil::AddUcToUcVector(data, (uchar)0);				//! [byte] 姿勢制御種別 0:
	ComUtil::AddUcToUcVector(data, (uchar)0);				//! [byte] リフタ制御種別

	ComUtil::AddFloatToUcVector(data, (float)600.0);			//! [float] 異常発生時間
	ComUtil::AddUcToUcVector(data, (uchar)0);				//! [byte] 自己位置同定   2:SLIMMING_METHOD
	ComUtil::AddUcToUcVector(data, (uchar)0);				//! [byte] 大域軌道修正
	ComUtil::AddFloatToUcVector(data, (float)10.0);			//! [float] 再計算閾値
	ComUtil::AddFloatToUcVector(data, (float)0.02);			//! [float] 位置精度
	ComUtil::AddFloatToUcVector(data, (float)0.1);			//! [float] 姿勢制度
	ComUtil::AddFloatToUcVector(data, (float)0.5);			//! [float] 近傍速度補正係数
	ComUtil::AddFloatToUcVector(data, (float)0.5);			//! [float] 近傍速度補正係数
	ComUtil::AddIntToUcVector(data, (int)0);				//! 待機場所・要素数
	//! 待機場所の情報(要素数が存在するなら)
		//! 地図ID
		//! 待機場所・要素数
		//! 位置・姿勢

	ComUtil::AddUcToUcVector(data, (uchar)3);				//! [byte] 障害物関係パラメータ・検知方法
	ComUtil::AddUcToUcVector(data, (uchar)0);				//! [byte] 障害物関係パラメータ・待ち方向
	ComUtil::AddFloatToUcVector(data, (float)0.5);			//! [float] 監視距離
	ComUtil::AddFloatToUcVector(data, (float)0.1);			//! [float] 検知距離

	ComUtil::AddUcToUcVector(data, (uchar)0);				//! [byte] 障害物関係パラメータ・周囲距離種別

	//! [float] 障害物関係パラメータ・周囲距離
	for(uint i = 0; i < 4; i++)										//! Ver-1-0-1-3 AI移動、基本移動拡張
	{
		ComUtil::AddFloatToUcVector(data, (float)0);			//! [float] 近傍速度補正係数
	}
	//! [float] 障害物関係パラメータ・ファイル名
	for (uint i = 0; i < 20; i++)									//! Ver-1-0-1-3 AI移動、基本移動拡張
	{
		ComUtil::AddUcToUcVector(data, (uchar)0);
	}
}




//! ****************************************************************************
//! 座標要求のコマンド送信と応答を受信する関数。
//! 実行後、m_pos_x,m_pos_y,m_rad,m_pos_zに情報が入る
//!
CommToTransRobot::DError CommToTransRobot::SendRequestPosCommand(void)
{

	CommandCode recvCommand;
	ByteArray recvData(5);

	NetIf::Result result;
	DError error;
	int cycle_num;

	//座標指令の送信処理
	AddSendingCommand(CommandCode(CommandCode::MASTER_COM, CommandCode::SUB_MASTER_REQ_POS));

	SendNextCommand();


	//座標応答の受信処理
	result = m_NetIfClient.Receive(&recvCommand, &recvData);
	cycle_num = 1;

	if (!(recvCommand.GetCode() == CommandCode::MASTER_COM && recvCommand.GetSubCode() == CommandCode::SUB_MASTER_REQ_POS)) {
		result = NetIf::RESULT_ERROR_SendEnq;
	}

	while (result != NetIf::RESULT_OK && cycle_num < RECV_CONFIRM_NUM)
	{
		cycle_num++;
		usleep(RECV_CONFIRM_CYCLE_MS * 1000); //1[s]受信待ち。
		result = m_NetIfClient.Receive(&recvCommand, &recvData);
		if (!(recvCommand.GetCode() == CommandCode::MASTER_COM && recvCommand.GetSubCode() == CommandCode::SUB_MASTER_REQ_POS)) {
			result = NetIf::RESULT_ERROR_SendEnq;
		}
	}


	if (result == NetIf::RESULT_OK) {
		error = NOT_ERROR;
	}
	else
	{
		error = ERROR;
	}

	if (error == ERROR)
	{
		return error;
	}

	error = RecvCommandProcess(recvCommand, recvData);

	return error ;
}


//! ****************************************************************************
//! 送信すべきコマンドがあることを確認する。
//! @retval 送信すべきコマンドがある場合true
bool CommToTransRobot::HasSendingCommand(void)
{
	return (m_nCurrentPush != m_nCurrentPop);
}

//! ****************************************************************************
//! 送信するコマンドを追加する。
//! @param[in] code 追加するコマンドコード
void CommToTransRobot::AddSendingCommand(CommandCode code)
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
CommandCode CommToTransRobot::GetCommand(void)
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
