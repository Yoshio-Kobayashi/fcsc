## daihen_ur5_moveit_configの取り扱い説明書

### 本パッケージの説明  
このパッケージは  
`$ roslaunch moveit_setup_assistant setup_assistant.launch `  
を実行することによって、自動的に生成されるパッケージである。

Moveit!はros上で動作する関節ロボットの動作計画を行うアプリケーション群を指す。  
そのMoveit!で利用する衝突判定や関節のグループ化等を行ってくれるのが  
moveit_setup_assistantである。

ロボットモデル(urdf等)を修正した際に衝突判定や適用する逆運動学ソルバ等を決定するときに用いる。  
具体的な修正方法に関しては、    
https://sites.google.com/site/robotlabo/time-tracker/ros/ros-manipulator   
を参考にすると良い。  
注意点としては修正を行う際は直接このパッケージを修正するのではなく、  moveit_setup_assistantを実行し、修正を行うこと。

`$ roslaunch daihen_ur5_moveit_config demo.launch`   
を実行することでRviz上でロボットモデルを表示させることが可能となっている。

追記(7/19)  
### daihen_ur5_moveit_configにあるdaihen_ur5_moveit_planning_excution.launchに関して  
fcsc_move_group.launchにて呼びだされているlaunchファイル。  
move_group.launchの起動とremapによるtopic名の再定義が行われている様子。  
再定義されたtopicに関してはまた追記を行う予定。