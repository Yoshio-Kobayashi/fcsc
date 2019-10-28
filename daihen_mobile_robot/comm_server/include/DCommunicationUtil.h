/*
 * DCommunicationUtil.h
 *
 *  Created on: 2016/06/29
 *      Author: bottechdev03
 */

#ifndef LIB_LIBDAIHENROBOT_D_INC_DCOMMUNICATIONUTIL_H_
#define LIB_LIBDAIHENROBOT_D_INC_DCOMMUNICATIONUTIL_H_

#include <DBasicTypes.h>

//! ***************************************************************************
//! ***************************************************************************
//! @class CommandCode
//! @brief コマンドコード、サブコードを示す。
//! ***************************************************************************
//! ***************************************************************************
class CommandCode
{
public:
	//! コマンドのコード一覧
	enum Code : int
	{
		NO_CODE							= 0x00,
		INITIALIZE						= 0x01,
		ROBOT_DETAILS					= 0x02,
		ORDER 							= 0x04,
		MASTER_COM						= 0x06,
		MASTER_ORDER					= 0x07,
	};

	//! CODE_INITIALIZE 0x01 のサブコード
	enum SubInitialize : int
	{
		SUB_INITIALIZE					= 0x01,
		SUB_ROBOT_PARAMETER_INITIALIZE 	= 0x02,
		SUB_ROBOT_INITIALIZE_MOVE		= 0x03,
		SUB_POSE_CALIBRATE				= 0x04,
		SUB_ENVIRONMENT_INITIALIZE		= 0x06,
	};


	//! CODE_ORDER 0x04 と MASTER_ORDER 0x07のサブコード
	enum SubOrder : int
	{
		SUB_DISPLAY						= 0x01,
		SUB_REQUEST_ONLINE 				= 0x02,
		SUB_ALLOCATE					= 0x03,
		SUB_ACTION						= 0x04,
		SUB_PREDICT_ACTION  			= 0x09,
		SUB_CAMERA_MOVE_BY_ACTION 		= 0x0a,
		SUB_CALIBRATION_ACTION			= 0x0b,
		SUB_ACTION_STATE_CONTROL		= 0x0c,
//##		SUB_DBROAD_START_ACTION			= 0x0d,
		SUB_DBROAD_START_ACTION			= 0x4d,
		SUB_DBROAD_END_ACTION			= 0x0e,
		SUB_WATCHWALL_START_ACTION		= 0x0f,
		SUB_WATCHWALL_END_ACTION		= 0x10,
		SUB_SUBCONVEYER_ACTION			= 0x11,
//##		SUB_MASTER_SWITCH_ACTION		= 0x12,
		SUB_MASTER_SWITCH_ACTION		= 0x0d
	};


	//! MASTER_COM 0x06 のサブコード(マスター装置通信用コマンド）
	enum SubMasterCom : uchar
	{
		SUB_MASTER_REQ_STATE					= 0x01,
		SUB_MASTER_REQ_POS						= 0x02,
		SUB_MASTER_ERROR_NOTIFICATION			= 0x03,
		SUB_MASTER_SUB_ACTION_FINISHED			= 0x04
	};


	//! コンストラクタ
	CommandCode(void)
	: m_Code(CommandCode::INITIALIZE)
	, m_ucSubCode(CommandCode::SUB_INITIALIZE)
	, m_ucSeqNo(0)
	{
		;
	}
	//! コンストラクタ
	CommandCode(Code code, uchar subCode)
	: m_Code(code)
	, m_ucSubCode(subCode)
	, m_ucSeqNo(m_sSeqNo)
	{
		m_sSeqNo++;
		if(m_sSeqNo < 0x80)
		{
			m_sSeqNo = 0x80;
		}
	}
	//! コンストラクタ
	CommandCode(Code code, uchar subCode, uchar seqNo)
	: m_Code(code)
	, m_ucSubCode(subCode)
	, m_ucSeqNo(seqNo)
	{
		;
	}
	//! コピーコンストラクタ
	CommandCode(const CommandCode& arg)
	: m_Code(arg.m_Code)
	, m_ucSubCode(arg.m_ucSubCode)
	, m_ucSeqNo(arg.m_ucSeqNo)
	{
		;
	}
	//! デストラクタ
	~CommandCode(void){;}

	//! 代入演算子
	CommandCode operator=(const CommandCode& arg)
	{
		m_Code = arg.m_Code;
		m_ucSubCode = arg.m_ucSubCode;
		m_ucSeqNo = arg.m_ucSeqNo;
		return *this;
	}

	//! 比較演算子
	bool operator==(const CommandCode& arg)
	{
		if((m_Code == arg.m_Code) && (m_ucSubCode == arg.m_ucSubCode))
		{
			return true;
		}
		return false;
	}

	//! 比較演算子
	bool operator!=(const CommandCode& arg)
	{
		if((m_Code == arg.m_Code) && (m_ucSubCode == arg.m_ucSubCode))
		{
			return false;
		}
		return true;
	}

	//! コードを取得する
	Code GetCode(void)const{return m_Code;}
	//! サブコードを取得する
	uchar GetSubCode(void)const{return m_ucSubCode;}
	//! シーケンス番号取得
	uchar GetSeqNo(void)const{return m_ucSeqNo;}
	//! シーケンス番号セット
	void SetSeqNo(uchar ucSeqNo){m_ucSeqNo = ucSeqNo;}

private:
	//!
	static uchar m_sSeqNo ;
	// コード
	Code 	m_Code;
	// サブコード
	uchar 	m_ucSubCode;
	//! シーケンス番号
	uchar	m_ucSeqNo;
};

#endif /* LIB_LIBDAIHENROBOT_D_INC_DCOMMUNICATIONUTIL_H_ */
