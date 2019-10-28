## fcsc_descriptionの取り扱い説明書

### 本パッケージの構成
    /fcsc_description
        /config
         --.yaml
        /image
         --.png
        /launch
         --.launch
        /media
            /materials
                /script
                 --.material
                /textures
                 --.png
        /mesh
            /dae
             --.dae
             --.JPG
            /ur5_kinect_mount
             --.stl
        /model
         --.urdf
        /scenes
         --.scene
        /script
         --.py
         --.sh
        /world
         --.world

### 各ディレクトリ毎の説明
#### config  
ここにはyamlファイルが入っており、  
yamlファイルとはコマンドを通してparamサーバに読み込まれるファイルである。  
    コントローラ設定やgazeboに表示させる棚やサンドウイッチのサイズ等を決定することが可能になっている。  
    現状では、sandwich_size.yaml,shelf_size.yamlは利用されていない様子。  
    他のファイルに関しては調査中。  
    追記  
    fcsc_other.launch で shelf_size.yamlとcontainer_size.yamlはloadされていることを確認。
    
#### image
各モデル(サンドウイッチ等)のモデルのtfなどをRvizに表示させたものをscしたもの。  
    特に利用することはない。

#### launch
 このディレクトリ内ではgazeboにオブジェクトを表示させるlaunchファイルがまとめられている。  
    構造としては、spawn_object_urdf.launch でgazebo上でのオブジェクトの表示及び各引数のデフォルトの設定(個数,姿勢等)を行い、  
    spawn_sandwich.launch 等のlaunchファイルで引数を入れることによって希望するオブジェクトの表示を行うことができる。

#### media
    /materials
    サンドウイッチの位置検出をを娘なう際に用いていたARマーカを保存しているフォルダ。

    /mesh
    サンドウイッチ等のオブジェクトに対して画像を貼り付けることが可能となるstl,daeファイルを保存しているフォルダ。
    
    /model
    gazebo上でロボット,オブジェクト等を表示させるために必要なurdfが保存されている。

    /scenes

    /script
    ARマーカの作成を行うプログラム。
    どこかで呼びだされているかどうか調査中。
    特に編集を行う必要はない。

    /world
    gazeboのworldを保存したファイル。
    読みこめば、保存されたworldを再現することが可能。
    ただし、かなり過去のものなので必要ではない。

### 現状で利用する可能性のあるlaunchファイルの説明
#### publish_camera_info
カメラ情報をパブリッシュするlaunchファイル。  
おそらくカメラのキャリブレーションを行う際に利用されていた？
利用したことがないので不明。  
    要検証。

#### spawn_objects_urdf.launch
各オブジェクトをgazebo内にスポーンさせるために必要な引数を統合したもの。  
    spawn_〜.launchを起動すると必ず呼び出される。  
各引数で姿勢や位置、出すオブジェクトを調整できるようになっている。  

#### spawn_two_shelves.launch 
2018年fcscにおける規定の棚２つを呼び出すことのできるlaunchファイル。  

#### spawn_sandwich.launch 
サンドウィッチを棚の中に表示させることのできるlaunchファイル。   
ただし、各サンドウイッチは棚座標基準で表示されるため、  
棚がgazebo内に存在していないと表示させることはできない。

#### spawn_sandwich_and_shelf.launch
2018年fcscにおける規定のサンドウィッチが陳列された棚とサンドウィッチが同時に呼び出すことができるlaunchファイル。  
サンドウィッチはspawn_sandwich.launchを用いて出力されている。

### ロボットモデル
gazebo上に出力されているロボットモデルはdaihen_ur5.urdf.xacroを読み込むことによって出力されている。  
xacroとはurdfのマクロ記述文法であり、xacroを用いることで多くの重複した文章を簡略化できるものである。  
参考  
<https://qiita.com/RyodoTanaka/items/174e82f06b10f9885265>  
<https://qiita.com/srs/items/43528d00ee789171367f>  

daihen_ur5.urdf.xacroでは自作または引用してきたロボットモデルを組み合わせることによって  
daihen_ur5を作っている。

    自作
    fcsc_description/model/my_ezgripper_dual_gen2.urdf.xacro　ハンド及びその結合部位
    fcsc_description/model/robot.urdf.xacro   台車
    fcsc_description/model/container.urdf.xacro   コンテナ
    fcsc_description/model/fcsc_gazebo.xacro  カメラやレーザとその結合部位

    引用: ur_description/urdf/ur5.urdf.xacro    ur5(ロボットアーム)

自作に関しては修正は可能だが、  
引用してきたモデルに関しては修正を行うと逆運動学等が正常に解けずに動かなくなる恐れがあるので  
修正は行わないこと。  
新たに連結部位などを作る際はまずその部位のurdfを作成した後、  
daihen_ur5.urfd.xacro内で結合させるのがベスト。
    
    
    