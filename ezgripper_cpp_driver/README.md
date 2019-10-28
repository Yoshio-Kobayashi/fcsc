EZgripperの動作テスト
==========================  

## 手順
1) ハンド電源用のコンセントを挿す  
2) XboxのコントローラーをPCに接続する  
3) EZGripperをPCに接続する  
4) 関連パッケージをインストール(インストール済みなら不要)  
```
sudo apt-get install ros-kinetic-joy
```  
5) コマンド実行  
ハンド2つを同時に動かす場合
```
roslaunch my_ezgripper joy2sync.launch
```

ハンド１つを動かす場合
```
roslaunch my_ezgripper joy.launch
```

6) ハンドをXboxコントローラで動かしてみる  

ボタン|動作  
:----:|:---:  
Yボタン|ハンドを閉じる(softClose トルク effort=20)  
Aボタン|ハンドを閉じる(HardClose トルク最大:effort=100)  
Bボタン|ハンドを開く  


## コントローラーを押しても反応しない場合  
デバイスドライバを確認する
```
ls /dev/input/js*
```

以下のような内容が表示されるはず
```
/dev/input/js0 /dev/input/js1
```  

確認したらlaunchファイルのdevのパラメータを変更する
