hierarchical_trajectory_database
=====================================
階層型軌道データベース[1]を実装しているパッケージ

<img src="./image/trajectory_database.png" width="700px"/>

## ライブラリの説明
### hierarchical_trajectory_database_handler
* 階層型軌道データベースの構築
* 階層型軌道データベースを利用した軌道生成

## ノードの説明
### generate_trajectory_server
#### サービス・サーバ
* generate_trajectory (hierarchical_trajectory_database/GenerateTrajectory)

## 設定
### ダウンロード
```
git clone https://github.com/takubolab/hierarchical_trajectory_database.git  
```

### 依存パッケージのダウンロード
```
git clone https://github.com/takubolab/trajectory_data_handler.git  
git clone https://github.com/takubolab/dmp.git
```

## 軌道データベース構築のための参照軌道の準備
[moveit::planning_interface::MoveGroupInterface::Plan](http://docs.ros.org/melodic/api/moveit_ros_planning_interface/html/structmoveit_1_1planning__interface_1_1MoveGroupInterface_1_1Plan.html)をbagファイルとして保存しておく．
Planをbagファイルとして保存するために trajectory_data_handler を使用している．

## 軌道生成サーバを立ち上げる
あらかじめ軌道データを１つのディレクトリに格納しておく．  
サンプルの軌道データを使用する場合は`nas/matsumoto/Data/trajectory`をフォルダごと適当な場所にコピーする．  
moveitを立ち上げる
```
roslaunch daihen_ur5_moveit_config demo.launch
```
サーバを立ち上げる
```
roslanch hierarchical_trajectory_database generate_trajectory_server.launch
```
### パラメータ
* trajectory_file_path (string)  
参照軌道のbagファイルを保存しているディレクトリのパス  
テスト用のデータを使用する場合は`[保存先までのパス]/trajectory/dataset_09/drink_filtered/`などと設定する．

## 使用するA*アルゴリズムを変更する
階層型軌道データベースでは各階層に状態遷移グラフが構築されており、グラフ内のノードはロボットアームの姿勢を表す．
この状態遷移グラフ内を探索することで軌道生成する．
探索アルゴリズムにはA\*を使用している．
修士論文執筆時はboostのastar_searchを使用していたがノード展開時にロボットアームの干渉チェックを追加できていなかったため、周辺の障害物に衝突するような軌道が生成されるという問題があった．
そのため、修論発表後にA\*アルゴリズムのノード展開時にロボットアームの干渉チェックを追加するため、boostのastar_searchから自作のA*に変更した．
boostのA\*に変更するときはhierarchical_trajectory_database.cppの以下の部分のコメントアウトを変更する．  

* インクールドファイルを変更する
my_astar_search.hが自作のA\*アルゴリズム．
```
#include "hierarchical_trajectory_database/my_astar_search.h"
// #include "hierarchical_trajectory_database/astar_search.h"
```

* 関数findPartialPath()内の以下の3箇所を変更する
my_astar::astar_searchが自作のA\*アルゴリズム．  
```
bool found_path = my_astar::astar_search(略
// bool found_path = astar_search(略
```


## 参考文献
[1]Deniša, M., & Ude, A. (2015). Synthesis of New Dynamic Movement Primitives Through Search in a Hierarchical Database of Example Movements. International Journal of Advanced Robotic Systems, 12(10), 1–14. https://doi.org/10.5772/61036
