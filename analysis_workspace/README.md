analysis workspace
=========================
ロボットアームのワークスペース解析パッケージ

## ライブラリ
### analyse_workspace
#### 機能
* 関節空間上でのワークスペース解析
関節範囲内でロボットアームの姿勢を逐次変更して順運動学+干渉チェックする．
どれだけの到達可能点があるのか確認する．
<strong>6自由度</strong>のロボットアームにしか対応していない．

* 作業空間上での到達可能点に関するワークスペース解析
直方体で作業範囲を設定して、その範囲内で指定した手先姿勢で逐次、逆運動学+干渉チェックする．
どれだけの到達可能点があるのか確認する．

* 作業空間上での軌道計画に関するワークスペース解析
直方体で作業範囲を設定して、その範囲内で指定した手先姿勢で逐次、軌道計画を行う．
目標手先位置・姿勢について軌道計画が成功するか確認する．

* 解析結果の保存
  * 到達可能点の保存
  * 成功率(作業空間上での到達可能点の割合)及び、到達可能点の分布についての平均、分散、の保存

## ノード
### test_analyse_workspace
* 陳列棚、コンテナ内でのワークスペース解析で使用したテストノード

## ワークスペース解析する
インクールドする
```
#include <analysis_workspace/analyse_workspace.h>
```

定義する．
```
WorkspaceAnalyzer workspace_analyzer(/* moveit上でのロボットアームのグループ名 */"manipulator");
```

### 作業空間上でワークスペース解析する
作業領域と手先姿勢を設定する．
なお、手先姿勢は複数設定可能．
```
moveit_msgs::WorkspaceParameters　task_space;
std::vector<geometry_msgs::Quaternion>  orientations;
```

ワークスペース解析する
```
workspace_analyzer.analyseWorkspaceFromTaskSpace(task_space, orientations);
```

### 関節空間上でワークスペース解析する
関節範囲のサンプリングの粒度(関節範囲を何分割するか)を設定して解析する．
```
int particle = 10;
workspace_analyzer.analyseWorkspaceFromJointSpace(particle);
```

### 解析結果を保存する
関数`analyseWorkspaceFrom~~()`を実行後に行う．
到達可能点を保存．

```
string file_name="aaaa"
workspace_analyzer.saveWorkspaceDataToDAT(file_name)
```
実行後、`WorkspaceData_[ファイル名]_[タイムスタンプ].dat`が`analyse_workspace/data/`に保存される．  </br>
成功率、到達可能点の分布の平均、分散を保存．
```
string file_name="aaaa"
workspace_analyzer.saveAnalysisResultToDAT(file_name)
```
実行後、`AnalysisResult_[ファイル名]_[タイムスタンプ].csv`が`analyse_workspace/data/`に保存される．  
