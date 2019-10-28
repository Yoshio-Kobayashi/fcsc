//============================================================================
// Name        : DummyTransRobot.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>

#include "NetIf.h"
#include "CommToArm.h"

using namespace std;


uchar CommandCode::m_sSeqNo = 0x80;

int main() {

	CommToArm commToArm;
	uint port = 10020;                      // 搬送ロボットのポート番号

	//TCPサーバー接続
	cout << "accept..." << endl;
	commToArm.Listen(port, 1000);
	commToArm.Accept();
	cout << "accept OK" << endl;

	while(1){
		commToArm.Execute();
		usleep(100 * 1000);
	}

	//TCP終了
	//commToArm.Close();

	return 0;
}
