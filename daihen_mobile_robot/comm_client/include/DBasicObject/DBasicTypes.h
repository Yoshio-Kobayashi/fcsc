/**
 * 基本要素をまとめたファイル
 *
 * @author H.Sakahara
 * @par 更新履歴
 * - 2016/12/19 H.Sakahara
 *  -# 新規作成
 */

#ifndef DBASICTYPES_H
#define DBASICTYPES_H

#include "float.h"
#include "string.h"
#include "stdio.h"
#include <cmath>
#include <vector>

#define LINUXLIB

typedef unsigned char 	uchar;
typedef unsigned short  ushort;
typedef unsigned int	uint;
typedef	unsigned long	ulong;
#ifndef LINUXLIB
typedef std::string		String;
#endif


//! @note syslsiのUcVectorクラスは以下のデータに
//!		  いくつかの関数を加えたクラス。互換あり
typedef std::vector<uchar> UcVector;

namespace BasicUnit
{
	//! 単位をmからmmに変換する際の係数
	const int m2mm = 1000;
	//! 単位をmmからmに変換
	const float mm2m = 0.001f;
	//! 単位をラジアンから度に変換
	const float rad2deg = 57.2957795f;
	//! 単位を度からラジアンに変換
	const float deg2rad = 0.0174532925f;
	//! 単位を秒からミリ秒に変換
	const int sec2ms = 1000;
	//! 単位をミリ秒から秒に変換
	const float ms2sec = 0.001f;
	//! 単位をミリ秒から秒に変換
	const float ms2min = 0.001f / 60.0f;

	//! 単位をマイクロ秒からミリ秒に変換
	const float us2ms  = 0.001f;
	//! 単位をマイクロ秒から秒に変換
	const float us2sec = 0.001f * 0.001f;
	//! 単位をマイクロ秒から分に変換
	const float us2min = 0.001f * 0.001f / 60.0f;
	//! 単位をm/secからmm/secに変換
	const float mps2mmps = 1000.0f;
	//! 単位をmm/secからm/secに変換
	const float mmps2mps = 0.001f;
	//! 単位をrad/secからrpmに変換
	const float radps2rpm = 9.54929659f;
	//! 単位をrpmからrad/secに変換
	const float rpm2radps = 0.104719755f;
	//! 2π
	const float M_2PI	= 6.28318531f;

	//! 微少量
	static const float fEps   = 0.00001f;
	//! ラジアンの微少量(同値とみなす場合などに使う）
	static const float fEps_rad = 0.0001f;
	//! 度の微少量(同値とみなす場合などに使う）
	static const float fEps_deg = 0.01f;
	//! 三角関数の結果の微少量(同値とみなす場合などに使う）
	static const float fEps_sin = 0.0001f;
	//! 距離の微少量
	static const float fEps_m   = 0.0001f;
	//! 角速度の微少量
	static const float fEps_radps = 0.00001f;
	//! 速さの微少量
	static const float fEps_mps   = 0.00001f;
	//! 時間の微少量
	
	//! イレギュラーな値（無効値）
	static const float fIrregularValue = 3.402823466e38f;
}

// 正負を示す。
template <typename _Tp> inline int sign(_Tp val)
{
	return 1 - (val <= 0) - (val < 0);
}

//! ***************************************************************************
//! ***************************************************************************
//! @class ID
//! @brief IDの番号と名前を持つ
//! ***************************************************************************
//! ***************************************************************************
class ID
{
public:
	static const int m_nNameLength = 20;

	//! デフォルトコンストラクタ
	inline		 ID(void);
	//! コンストラクタ、番号と名前で初期化する。
	inline		 ID(int nNo, const char* szName);
	//! コンストラクタ。通信のデータ列から取得する。
	inline ID(uint* punNextNum, const UcVector& array, uint unStartNum);
	//! コピーコンストラクタ
	inline ID(const ID& arg);
	//! デストラクタ
	inline ~ID(void);

	//! 通信のデータ列へデータを埋め込む
	inline void AddToUcVector(UcVector* array)const;
	//! 代入演算子
	inline const ID& operator=(const ID& arg);
	//! 等号演算子
	inline bool operator==(const ID& arg)const;
	//! 番号を取得する。
	inline int GetNo(void)const;
	//! 名前を取得する。
	inline const char* GetName(void)const;
	//! 番号と名前を設定する。
	inline void Set(int nNo, const char* szName);

private:
	//! 番号
	int	m_nNo;
	//! 名前
	char m_szName[m_nNameLength];
};

//! ***************************************************************************
//! ***************************************************************************
//! @class ComUtil
//! @brief 通信データ取り出し、入れ込みライブラリ
//! ***************************************************************************
//! ***************************************************************************
class ComUtil
{
public:
	//! バイト配列に変換する
	static void AddUcToUcVector(UcVector* array, uchar ucAddData);
	//! バイト配列に変換する
	static void AddIntToUcVector(UcVector* array, int nAddData);
	//! バイト配列に変換する
	static void AddUintToUcVector(UcVector* array, uint unAddData);
	//! バイト配列に変換する
	static void AddFloatToUcVector(UcVector* array, float fAddData);
	//! バイト配列から取得する
	static char GetChar
			(uint* punEndNum, const UcVector& array, uint unStartNum);
	//! バイト配列から取得する
	static int 	GetInt
			(uint* punEndNum, const UcVector& array, uint unStartNum);
	//! バイト配列から取得する
	static uint	GetUint
			(uint* punEndNum, const UcVector& array, uint unStartNum);
	//! バイト配列から取得する
	static float GetFloat
			(uint* punEndNum, const UcVector& array, uint unStartNum);
};
		
//! ***************************************************************************
//! ***************************************************************************
//! @class Range
//! @brief 上限と下限を持つ幅
//! ***************************************************************************
//! ***************************************************************************
class Range
{
public:
	//! デフォルトコンストラクタ
	inline Range(void);
	//! コンストラクタ　上限と下限で初期化する。
	inline Range(float fEdge1, float fEdge2);

	//! コンストラクタ。通信のデータ列から取得する。
	inline Range
			(uint* punNextNum, const UcVector& array, uint unStartNum);
	//! コピーコンストラクタ
	inline Range(const Range& arg);
	//! デストラクタ
	inline ~Range(void);

	//! 通信のデータ列へデータを埋め込む
	inline void AddToUcVector(UcVector* array)const;
	//! 代入演算子
	inline const Range& operator=(const Range& arg);
	//!
	inline float GetUpperLimit(void)const;
	//!
	inline float GetLowerLimit(void)const;

	inline bool IsIn(float fValue)const;

private:
	float m_fUpperLimit;
	float m_fLowerLimit;
};

class TimeUtil
{
public:
	static long GetDifference_sec(const struct timeval& before, const struct timeval& after);
	static long GetDifference_ms(const struct timeval& before, const struct timeval& after);
	static long GetDifference_us(const struct timeval& before, const struct timeval& after);

};

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// 以下インライン関数の定義
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//	IDのinline関数群
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//! ****************************************************************************
//! デフォルトコンストラクタ
ID::ID(void)
: m_nNo(0)
{
	memset(m_szName, 0, m_nNameLength);;
}

//! ****************************************************************************
//! コンストラクタ
ID::ID(int nNo, const char* szName)
: m_nNo(nNo)
{
	memset(m_szName, 0, m_nNameLength);
#ifdef LINUXLIB
	strncat(m_szName, szName, m_nNameLength);
	m_szName[m_nNameLength-1] = '\0';
#else
	strncat(m_szName, szName, m_nNameLength);
	m_szName[m_nNameLength-1] = '\0';
#endif
}

//! ****************************************************************************
//! コンストラクタ
ID::ID(const ID& arg):m_nNo(arg.m_nNo)
{
	memset(m_szName, 0, m_nNameLength);
#ifdef LINUXLIB
	strncat(m_szName, arg.m_szName, m_nNameLength);
#else
	strncat_s(m_szName, arg.m_szName, m_nNameLength);
	m_szName[m_nNameLength-1] = '\0';
#endif
}

//! ****************************************************************************
//! コンストラクタ。通信のデータ列から取得する。
//! @param[out] punNextNum	コンストラクタで使ったデータの次のデータの値。
//! @param[in]  array		データ列
//! @param[in]  unStartNum	コンストラクタで使う最初のデータの値
ID::ID(uint* punNextNum, const UcVector& array, uint unStartNum)
: m_nNo(0)
{
	memset(m_szName, 0, m_nNameLength);
	m_nNo = ComUtil::GetInt(punNextNum, array, unStartNum);
	for(int i = 0; i < m_nNameLength; i++)
	{
		m_szName[i] = array[i+4];
	}
	*punNextNum += m_nNameLength;
}

//!*****************************************************************************
//! 通信のデータ列へデータを埋め込む
//! @param[out] array
void ID::AddToUcVector(UcVector* array)const
{
	ComUtil::AddIntToUcVector(array, m_nNo);
	for(int i = 0; i < m_nNameLength; i++)
	{
		array->push_back(m_szName[i]);
	}
}

//! ****************************************************************************
//! 代入演算子
const ID& ID::operator=(const ID& arg)
{
	m_nNo = arg.m_nNo;
#ifdef LINUXLIB
	strncat(m_szName, arg.m_szName, m_nNameLength);
#else
	strncat_s(m_szName, arg.m_szName, m_nNameLength);
#endif
	m_szName[m_nNameLength-1] = '\0';
	return *this;
}

//! ****************************************************************************
//! 等号演算子
inline bool ID::operator==(const ID& arg)const
{
	if(this->m_nNo != arg.m_nNo)
	{
		return false;
	}
	for(int i = 0; i < m_nNameLength; i++)
	{
		if(m_szName[i] != arg.m_szName[i])
		{
			return false;
		}
	}
	return true;
}

//! ****************************************************************************
//! デストラクタ
ID::~ID(void)
{
	;
}

//! ****************************************************************************
//! 番号と名前を設定する。
void ID::Set(int nNo, const char* szName)
{
	m_nNo = nNo;
#ifdef LINUXLIB
	strncpy(m_szName, szName, m_nNameLength);
#else
	strncpy_s(m_szName, szName, m_nNameLength);
#endif
	m_szName[m_nNameLength-1] = '\0';
}

//! ****************************************************************************
//! 番号を取得する。
int ID::GetNo(void)const
{
	return m_nNo;
}

//! ****************************************************************************
//! 名前を取得する。
const char* ID::GetName(void)const
{
	return m_szName;
}

//! ****************************************************************************
//! デフォルトコンストラクタ
Range::Range(void)
: m_fUpperLimit(0)
, m_fLowerLimit(0)
{
	;
}

//! ****************************************************************************
//! コンストラクタ　上限と下限で初期化する。
Range::Range(float fEdge1, float fEdge2)
: m_fUpperLimit(fEdge1)
, m_fLowerLimit(fEdge2)
{
	if(fEdge1 < fEdge2)
	{
		m_fUpperLimit = fEdge2;
		m_fLowerLimit = fEdge1;
	}
}

//! ****************************************************************************
//! コンストラクタ。通信のデータ列から取得する。
Range::Range(uint* punNextNum, const UcVector& array, uint unStartNum)
{
	m_fUpperLimit = ComUtil::GetFloat(punNextNum, array, unStartNum);
	m_fLowerLimit = ComUtil::GetFloat(punNextNum, array, *punNextNum);
}

//! ****************************************************************************
//! 通信のデータ列へデータを埋め込む
void Range::AddToUcVector(UcVector* pArray)const
{
	ComUtil::AddFloatToUcVector(pArray, m_fUpperLimit);
	ComUtil::AddFloatToUcVector(pArray, m_fLowerLimit);
}

//! ****************************************************************************
//! コピーコンストラクタ
Range::Range(const Range& arg)
: m_fUpperLimit(arg.m_fUpperLimit)
, m_fLowerLimit(arg.m_fLowerLimit)
{

}

//! ****************************************************************************
//! デストラクタ
Range::~Range(void)
{

}

//! ****************************************************************************
//! 代入演算子
const Range& Range::operator=(const Range& arg)
{
	m_fUpperLimit = arg.m_fUpperLimit;
	m_fLowerLimit = arg.m_fLowerLimit;
	return *this;
}

//! ****************************************************************************
//! 代入演算子
inline float Range::GetUpperLimit(void)const
{
	return m_fUpperLimit;
}

//! ****************************************************************************
//! 代入演算子
inline float Range::GetLowerLimit(void)const
{
	return m_fLowerLimit;
}

//! ****************************************************************************
//! 
inline bool Range::IsIn(float fValue)const
{
	return m_fLowerLimit < fValue && fValue < m_fUpperLimit;
}

#endif
