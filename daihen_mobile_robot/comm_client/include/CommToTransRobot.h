/*
 * CommToTransRobot.h
 *
 *  Created on: 2018/06/08
 *      Author: M.Kubo
 */

#ifndef COMMTOTRANSROBOT_H_
#define COMMTOTRANSROBOT_H_

#include "NetIf.h"
#include "DCommunication/DCommunicationUtil.h"
#include "DBasicObject/DBasicTypes.h"



class CommToTransRobot
{
public:

	enum DError : int
	{
		NOT_ERROR = 0,
			ERROR = 1
	};

	//! 動作系アクションのメイン権限持ち
	enum OrderDevice :uchar
	{
		SERVER = 1,			//サーバー
		MASTER_DEVICE = 2	//マスター装置
	};


	CommToTransRobot();
	~CommToTransRobot(){};


	bool Connect(const char* pServerIpAddress, uint unServerPort);
	void Close();

	void Execute(void);

	void AddSendingCommand(CommandCode code);

	DError SendMoveActionCommand(float x, float y, float rad, float z);	//移動用の関数（ロボット座標の相対値）
	DError SendRequestPosCommand(void);    //座標を要求する関数

	inline float GetPosX(void)const {return m_pos_x;}
	inline float GetPosY(void)const {return m_pos_y;}
	inline float GetRad (void)const {return m_rad;}
	inline float GetPosZ(void)const {return m_pos_z;}

private:


	DError RecvCommandProcess(const CommandCode& recvCommand, const ByteArray& recvData) ;

	DError RecvOrderCommand(const CommandCode& command,const ByteArray& data);
	DError RecvMasterDeviceCommand(const CommandCode& command,const ByteArray& data);

	DError SendNextCommand();	//!キューにあるコマンドの送信
	DError SendOrderCommand (const CommandCode& command);

	void GenMoveActionData(ByteArray* data);

	bool HasSendingCommand(void);
	CommandCode GetCommand(void);

	const float RECV_CONFIRM_CYCLE_MS = 100 ; //受信確認周期
	// const float RECV_CONFIRM_NUM = 100 ; 	  //受信確認回数
	const float RECV_CONFIRM_NUM = 600 ; 	  //受信確認回数

	OrderDevice m_orderDevice ;  // 上位指令状態
	uchar m_masterID ;           // マスター装置ID
	float m_ctrlSig ;            // 制御用信号
	uchar m_error ;              // エラー信号

	//現在位置
	float m_pos_x ;				// X座標
	float m_pos_y ; 			// Y座標
	float m_rad   ;				// 姿勢[rad]
	float m_pos_z ;				// リフタ高

	//直値移動の相対移動値保存
	float m_dmove_pos_x ;				// X座標
	float m_dmove_pos_y ; 			// Y座標
	float m_dmove_rad   ;				// 姿勢[rad]
	float m_dmove_pos_z ;				// リフタ高


	//キュー
	static const int	nMaxQueue= 100;				//! コマンドキューの最大値
	int					m_nCurrentPush;				//! 現在のpush位置
	int					m_nCurrentPop;				//! 現在のpull位置
	CommandCode 		commandQueue[nMaxQueue];	//! コマンドキュー

	NetIfClient m_NetIfClient;

};

#endif /* COMMTOTRANSROBOT_H_ */
