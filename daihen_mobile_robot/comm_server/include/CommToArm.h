/**
 * マスター装置と通信するクラス
 * 
 * @author M.Kubo
 * @par 更新履歴
 * - 2018/05/31 M.Kubo
 *  -# 新規作成
 */

#ifndef DCOMM_TO_MASTER_DEVICE_H
#define DCOMM_TO_MASTER_DEVICE_H

#include <DBasicTypes.h>
#include <DCommunicationUtil.h>
#include <stdio.h>
#include "NetIf.h"


class RobotBaseShared;
class ReceiveDataShared;
class ActionControlShared;
class LocalizationShared;
class TestMasterDeviceActionData;
class MasterSwitchActionData;

//! ***************************************************************************
//! ***************************************************************************
//! @class CommToMasterDeviceTherver
//! @brief マスター装置との通信スレッド
//! ***************************************************************************
//! ***************************************************************************
class CommToArm
{
public:

	enum DError : int
	{
		NOT_ERROR = 0,
			ERROR = 1
	};
	CommToArm(void);		//! コンストラクタ
	~CommToArm(void);			//! デストラクタ

	bool HasSendingCommand(void);				//! 送信すべきコマンドがあることを確認する。
	void AddSendingCommand(CommandCode code);	//! 送信するコマンドを追加する。
	CommandCode GetCommand(void);				//! キューからコマンドを抜き出す。

	bool Listen(int nPort, int nBackLog){return m_NetIfServer.Listen( nPort, nBackLog);};
	bool Accept(void){return m_NetIfServer.Accept();};
	void Execute();

private:

	NetIfListener			m_NetIfServer;					//! 通信インターフェース

	//現在位置
	float m_pos_x ;				// X座標
	float m_pos_y ; 			// Y座標
	float m_rad   ;				// 姿勢[rad]
	float m_pos_z ;				// リフタ高

	//キュー
	static const int	nMaxQueue= 100;				//! コマンドキューの最大値
	int					m_nCurrentPush;				//! 現在のpush位置
	int					m_nCurrentPop;				//! 現在のpull位置
	CommandCode 		commandQueue[nMaxQueue];	//! コマンドキュー

//	const float REQ_TIMEOUT = 5.0 ;
//
//	void CreateNewCommand(void);
	void HandleCommand(const CommandCode& recvCommand, const ByteArray& recvData);
	DError HandleOrderCommand(const CommandCode& command,const ByteArray& data);
	DError HandleMasterDeviceCommand(const CommandCode& command,const ByteArray& data);	//! マスター装置の通信コマンドを受信する。
	DError SendNextCommand();	//!キューにあるコマンドの送信
	DError SendOrderCommand (const CommandCode& command);

};

#endif
