## fcsc_launchの取扱説明書

### 本パッケージの概要
このパッケージにはシミュレーション及び、実機でdaihen_ur5を稼働させる際に利用するファイルが格納されているパッケージである。  

### 各launchファイルの説明
#### daihen_ur5_simple_simulator.launch
#### daihen_ur5_simulator.launch
シミュレーション上でdaihen_ur5を動かすためのlaunchファイル。  
simple_simulatorではsimulatorとは異なり、いくつか起動していないlaunchがある。  
(カメラ関連や陳列する商品を出す等)  
アームを動かして物体を持ち上げる等の際はsimple_simulatorで良いが、  
fcsc本番同様の動作をシミュレーション上で行う際には  
simulatorを使うことにより必要なlaunchファイルをすべて起動してくれる模様。

#### ezgripper_bringup.launch 
ur5に取り付けているハンドを起動するためのlaunchファイル。  
実機実験を行う際に利用する。

#### fcsc_move_group.launch
daihen_ur5のmoveit!を起動するlaunchファイル。  
これも実機実験で利用するものになっている。  
daihen_ur5_moveit_configにあるdaihen_ur5_moveit_planning_excution.launchを呼び出している。

#### fcsc_other.launch
Realsenseを用いた認識動作に関するlaunchファイルと廃棄するサンドウィッチのマーカid,棚のサイズなどを読み込んでいる。  
主に認識関連の動作に関連するyamlファイルやオブジェクトのサイズなどをrosparamによって読み込んでいる。   
また、tf情報のbloadcartを行うnodeも起動している。 
tf関連のnodeに関してはfcsc_tfを参照すること。

#### my_ur_common.launch
ur_common.launch(https://github.com/ros-industrial/ur_modern_driver/blob/master/launch/ur_common.launch)  
に修正を加えたもの。  
修正内容に関してはrosparam paramによってsource_listの設定を行っている。

#### realsense_bringup.launch
実機で動作させる際にrealsenseを立ち上げるためのlaunchファイル。  
実機以外で利用することはないと思われる。  

#### ur5_bringup.launch
#### ur5_bringup_joint_limited.launch
ur5のドライバの起動を行うlaunchファイル。    
実機実験の時に利用する。  
具体的な違いに関しては関節の可動範囲が制限されているかどうかである。   
実機においてはrealsense等の配線の都合上、各関節が回りすぎてしまうと配線を巻き込んでしまう場合があるので、  
実機を利用する際はjoint_limitedの方を利用する。
