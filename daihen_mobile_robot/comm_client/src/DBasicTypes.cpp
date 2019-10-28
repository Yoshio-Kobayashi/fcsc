/*
 * DBasicTypes.cpp
 *
 *  Created on: 2016/07/24
 *      Author: bottechdev03
 */

#include "DBasicObject/DBasicTypes.h"
#include <sys/time.h>

//! ****************************************************************************
//! バイト配列に変換する
void ComUtil::AddUcToUcVector(UcVector* array, uchar ucAddData)
{
	int* npAddData = (int*)(&ucAddData);
	array->push_back((*npAddData >> 0)& 0xff);
}

//! ****************************************************************************
//! バイト配列に変換する
void ComUtil::AddIntToUcVector(UcVector* array, int nAddData)
{
	array->push_back((nAddData >>24)& 0xff);
	array->push_back((nAddData >>16)& 0xff);
	array->push_back((nAddData >> 8)& 0xff);
	array->push_back((nAddData >> 0)& 0xff);
}

//! ****************************************************************************
//! バイト配列に変換する
void ComUtil::AddUintToUcVector(UcVector* array, uint unAddData)
{
	int* npAddData = (int*)(&unAddData);
	array->push_back((*npAddData >>24)& 0xff);
	array->push_back((*npAddData >>16)& 0xff);
	array->push_back((*npAddData >> 8)& 0xff);
	array->push_back((*npAddData >> 0)& 0xff);
}

//! ****************************************************************************
//! バイト配列に変換する
void ComUtil::AddFloatToUcVector(UcVector* array, float fAddData)
{
	int* npAddData = (int*)(&fAddData);
	array->push_back((*npAddData >>24)& 0xff);
	array->push_back((*npAddData >>16)& 0xff);
	array->push_back((*npAddData >> 8)& 0xff);
	array->push_back((*npAddData >> 0)& 0xff);
}

//! ****************************************************************************
//! バイト配列から取得する
char ComUtil::GetChar
		(uint* punEndNum, const UcVector& array, uint unStartNum)
{
	char cRetNum = array[unStartNum];
	*punEndNum = unStartNum+1;
	return cRetNum;
}

//! ****************************************************************************
//! バイト配列から取得する
int ComUtil::GetInt
		(uint* punEndNum, const UcVector& array, uint unStartNum)
{
	int nRetNum =
			(((int)array[unStartNum  ] << 24) & 0xff000000) |
			(((int)array[unStartNum+1] << 16) & 0x00ff0000) |
			(((int)array[unStartNum+2] <<  8) & 0x0000ff00) |
			(((int)array[unStartNum+3] <<  0) & 0x000000ff) ;
	*punEndNum = unStartNum+4;
	return nRetNum;
}

//! ****************************************************************************
//! バイト配列から取得する
uint ComUtil::GetUint
		(uint* punEndNum, const UcVector& array, uint unStartNum)
{
	int unRetNum =
			(((int)array[unStartNum  ] << 24) & 0xff000000) |
			(((int)array[unStartNum+1] << 16) & 0x00ff0000) |
			(((int)array[unStartNum+2] <<  8) & 0x0000ff00) |
			(((int)array[unStartNum+3] <<  0) & 0x000000ff) ;
	uint* punRetNum = (uint*)(&unRetNum);
	*punEndNum = unStartNum+4;
	return *punRetNum;
}

//! ****************************************************************************
//! バイト配列から取得する
float ComUtil::GetFloat
	(uint* punEndNum, const UcVector& array, uint unStartNum)
{
	int nRetNum =
			(((int)array[unStartNum  ] << 24) & 0xff000000) |
			(((int)array[unStartNum+1] << 16) & 0x00ff0000) |
			(((int)array[unStartNum+2] <<  8) & 0x0000ff00) |
			(((int)array[unStartNum+3] <<  0) & 0x000000ff) ;
	float* pfRetNum = (float*)(&nRetNum);
	*punEndNum = unStartNum+4;
	return *pfRetNum;
}


long TimeUtil::GetDifference_sec(const struct timeval& before, const struct timeval& after)
{
	return GetDifference_ms(before, after) / 1000;
}

long TimeUtil::GetDifference_ms(const struct timeval& before, const struct timeval& after)
{
	return ((after.tv_sec * 1000 + after.tv_usec / 1000)
			- (before.tv_sec * 1000 + before.tv_usec / 1000));
}

long TimeUtil::GetDifference_us(const struct timeval& before, const struct timeval& after)
{
	return (after.tv_sec * 1000000 + after.tv_usec)
			- (before.tv_sec * 1000000 + before.tv_usec);
}
