# robot-nav-viewer

`robot-nav-viewer` は、ブラウザ上で、ロボットの自動走行に使う、経路計画、経路追従などのアルゴリズムが機能するか、3Dマップを用いて試行出来るビューアーです。

現在は主に次のロボットを扱います。

- `Unitree Go2`
- `TurtleBot3 Burger`

研究用として、ロボット、経路生成、追従制御や、3DTilesデータの大規模マップを入れ替えることができます。

# URL

WebGLで動いており、ブラウザベースで使用できます。
**[Robot-nav-viewer Demo url](https://gou-asaga.github.io/robot-nav-viewer/web/)**

## スクリーンショット

### ロボット操作画面

<img width="1917" height="910" alt="viewer" src="https://github.com/user-attachments/assets/2161b38f-4cd0-40aa-b2bc-83e43c8245d4" />

### 経路生成など

<img width="1910" height="898" alt="planner" src="https://github.com/user-attachments/assets/5af936f0-4beb-439d-a598-8ff2a44fc174" />

### マップ編集画面

<img width="1916" height="903" alt="editor" src="https://github.com/user-attachments/assets/7ed41186-1846-4f51-9ce8-0d98e62a1aa7" />

### 3D都市モデルデータの描画（渋谷地下など）
<img width="1912" height="946" alt="image" src="https://github.com/user-attachments/assets/fcfee603-be3e-4753-8dc5-06161b9eeefe" />

### 3D都市モデル上での操作
<img width="1915" height="935" alt="image" src="https://github.com/user-attachments/assets/9b8c0a43-707b-432c-84e2-7a439201856a" />


## 主な機能

- Three.js による 3D ロボットビューアー
- Go2 と TurtleBot3 Burger の切り替え表示
- `WASD` による手動移動、左右キーによる yaw 回転、ジャンプ操作
- 障害物を避ける経路計画
- 経路生成過程を確認する 2D デバッグビュー
- Go2 用の折れ線 waypoint 追従
- TurtleBot3 用の曲線経路追従
- 別画面の 3D Map Editor
- 障害物の追加、削除、移動、リサイズ、高さ調整
- ロボット初期スポーン位置の編集
- PLATEAU 3D Tiles を読み込む 3D都市マップモード
- Shibuya / Shinjuku / Chiyoda の PLATEAU プリセット選択
- 3D Tiles 表面へのロボットスポーンと手動走行
- 3D Tiles 表面上での開始点、目的地設定、3D経路計画、経路追従
- 階段の直進判定、同一フロア用の軽量探索、3D探索フォールバックを組み合わせた 3D Tiles 用 planner
- 3D Tiles 経路探索の状態を確認する `3D Route Search` デバッグビュー
- 自由視点の中心レティクル表示と、ダブルクリックによる orbit 中心移動

## 画面

### Viewer

- ファイル: `web/index.html`
- 通常の簡易3Dマップ、ロボット操作、経路計画、経路追従を行う画面です。

### Map Editor

- ファイル: `web/editor.html`
- 障害物、マップサイズ、ロボットのスポーン位置を編集する画面です。
- Viewer とは別画面として開き、`Close` で戻ります。

### 3D Tiles / PLATEAU Viewer

- ファイル: `web/tiles.html`
- PLATEAU などの 3D Tiles データを読み込み、都市モデル上にロボットをスポーンして動かす画面です。
- 通常 Viewer とは状態を分け、3D Tiles 用の読み込み、カメラ、スポーン、経路計画、経路追従を扱います。

## 起動方法

リポジトリのルートでローカル HTTP サーバーを起動します。

```bash
python -m http.server 8000
```

通常 Viewer:

```text
http://localhost:8000/web/
```

Map Editor:

```text
http://localhost:8000/web/editor.html
```

3D Tiles / PLATEAU Viewer:

```text
http://localhost:8000/web/tiles.html
```

## 必要なもの

- Python 3
- WebGL 対応ブラウザ
- インターネット接続
- CDN から読み込む `three` / `three/addons`

推奨ブラウザ:

- Google Chrome
- Microsoft Edge
- Firefox

## 操作

### ロボット移動

- `W`: 前進
- `A`: 左移動
- `S`: 後退
- `D`: 右移動
- `←`: 左 yaw 回転
- `→`: 右 yaw 回転
- `F`: スプリント
- `Shift`: スニーク
- `Space`: ジャンプ

### カメラ

- `Camera`: ロボット追従視点 / 自由視点の切り替え
- 自由視点の `LMB drag`: 画面中央のレティクルを中心に orbit
- 自由視点の `Double Click`: クリックした地面や 3D Tiles 表面へ orbit 中心を移動
- 自由視点の `RMB drag`: pan
- `Wheel`: zoom
- `E`: 操作ガイドの表示 / 非表示

## 経路計画と追従

Viewer では `Start`、`Goal`、`Plan`、`Auto` を使って経路計画と自律移動を実行します。

基本の流れ:

1. `Start` で開始点を置く
2. `Goal` で目的地を置く
3. `Plan` で障害物を避ける経路を生成する
4. `Auto` で経路追従を開始する

ロボットごとの追従方針:

- Go2 は緑の折れ線経路を使い、折れ曲がり地点を waypoint として順番に通過します。
- Go2 は直線区間では直進し、折れ曲がり地点で yaw を調整します。
- TurtleBot3 は曲線経路を使い、滑らかに旋回しながら追従します。

経路計画の可視化:

- `Plan` 実行時に、右側のデバッグパネルでノード、エッジ、障害物、安全範囲、最終経路を確認できます。
- 2D デバッグマップは簡易マップの状態をもとに描画されます。

## Map Editor

Map Editor では 3D 表示のまま障害物を編集できます。

できること:

- 障害物ボックスの追加
- 障害物ボックスの削除
- 障害物ボックスの選択
- ドラッグによる移動
- 3D ハンドルによる幅、奥行き、高さの調整
- 障害物の elevation 調整
- マップサイズの変更
- ロボットの初期スポーン位置と yaw の設定

編集内容はブラウザの `localStorage` に保存され、Viewer と Editor で共有されます。

## PLATEAU / 3D Tiles モード

`web/tiles.html` では 3D Tiles の `tileset.json` を読み込み、都市モデルを表示できます。

現在のプリセット:

- `Shibuya`: `web/data/plateau/shibuya/tileset.json`
- `Shinjuku`: `web/data/plateau/shinjuku/tileset.json`
- `Chiyoda`: `web/data/plateau/chiyoda/tileset.json`

使い方:

1. `web/tiles.html` を開く
2. プルダウンで `Shibuya`、`Shinjuku`、`Chiyoda` のいずれかを選ぶ
3. `Load Tileset` を押す
4. `Spawn Robot` を押す
5. 3D Tiles の床や地面をクリックしてロボットをスポーンする
6. `WASD` と左右キーでロボットを動かす

右上の `POS` ボタンを押すと、ロボットの現在位置を表示できます。

3D Tiles 上でも `Start`、`Goal`、`Plan`、`Auto` を使えます。

- `Start` / `Goal`: 3D Tiles の床や階段上に経路点を置きます。
- `Plan`: 3D Tiles の表面、段差、壁判定を使って経路を作ります。
- `Auto`: 生成した経路をGo2の折れ線追従で進みます。
- `3D Route Search`: 探索済みノード、raw経路、最終経路、使用した探索モードを表示します。

3D Tiles 用の経路計画は、軽い判定から順に試します。

1. `short-step`: 目の前の1段段差を直進できるか確認する
2. `stair-line`: 複数段の階段を直線で登れるか確認する
3. `direct`: 通常の直線移動ができるか確認する
4. `flat`: 同一フロアを軽量な2Dグリッド探索で解く
5. `3d`: 段差や複雑な形状がある場合だけ3D探索にフォールバックする

### データの置き方

3D Tiles は `tileset.json` と、そこから参照される `data/` フォルダなどの相対パスを保ったまま置く必要があります。

例:

```text
web/data/plateau/shibuya/tileset.json
web/data/plateau/shibuya/data/...
```

プリセット以外の 3D Tiles を使う場合は、入力欄に任意の `tileset.json` URL を入れて `Load Tileset` を押します。

## PLATEAUデータについて

PLATEAU の 3D都市モデルはオープンデータとして公開されています。利用時は、配布元のライセンス、出典表記、利用条件を確認してください。

参考:

- PLATEAU: https://www.mlit.go.jp/plateau/
- G空間情報センター PLATEAUポータル: https://front.geospatial.jp/plateau_portal_site/
- 3D都市モデル標準製品仕様書・利用ルール: https://www.mlit.go.jp/plateau/libraries/

## 主なファイル

- `web/index.html`: 通常 Viewer の UI
- `web/viewer.js`: 通常 Viewer のメイン実装
- `web/viewer-main.js`: 通常 Viewer のエントリーポイント
- `web/editor.html`: Map Editor の UI
- `web/editor-main.js`: Map Editor のエントリーポイント
- `web/tiles.html`: 3D Tiles / PLATEAU Viewer の UI
- `web/tiles-main.js`: 3D Tiles / PLATEAU Viewer のエントリーポイント
- `web/tiles-viewer.js`: 3D Tiles 用 Viewer 実装
- `web/modules/map-state.js`: マップ状態の保存、読み込み
- `web/modules/path-planners.js`: 経路計画モジュール
- `web/modules/path-followers.js`: 経路追従モジュール
- `web/modules/planner-debug-view.js`: 経路計画デバッグビュー
- `web/modules/tiles-route-planner.js`: 3D Tiles 用の段差対応経路計画モジュール
- `web/assets/go2/`: Go2 関連アセット
- `web/assets/turtlebot3/`: TurtleBot3 関連アセット
- `web/data/plateau/`: PLATEAU / 3D Tiles データ配置場所
- `config/go2_map.json`: 初期マップ設定

## モジュール差し替えについて

このビューアーは、経路計画や経路追従の方式を比較しやすいように、段階的にモジュール分離しています。

現在の主な差し替え対象:

- 経路計画: `web/modules/path-planners.js`
- 3D Tiles 経路計画: `web/modules/tiles-route-planner.js`
- 経路追従: `web/modules/path-followers.js`
- 経路計画の可視化: `web/modules/planner-debug-view.js`
- マップ状態管理: `web/modules/map-state.js`

### 経路計画モジュールの入出力

入力:

- `start`: 開始点 `{ x, z }`
- `goal`: 目的地 `{ x, z }`
- `obstacleBounds`: 障害物の2D境界矩形
- `mapWidth`: マップ幅
- `mapHeight`: マップ高さ
- `clearance`: ロボット半径や安全余白

出力:

- `ok`: 経路生成に成功したか
- `path`: 折れ線経路 `[{ x, z }, ...]`
- `smoothPath`: 曲線追従用のサンプル点列
- `polylineMetrics`: 折れ線経路の距離情報
- `smoothPathMetrics`: 曲線経路の距離情報
- `debugData`: 可視化用のノード、エッジ、障害物、安全範囲など
- `reason`: 失敗理由

この形を守れば、Visibility Graph 以外の planner も比較しやすくなります。

### 3D Tiles 経路計画モジュールの入出力

`web/modules/tiles-route-planner.js` は、PLATEAU などの3D Tiles表面を歩くための planner です。通常の簡易マップとは違い、床高さと壁判定を関数として受け取ります。

入力:

- `start`: 開始点 `{ x, z, y }`
- `goal`: 目的地 `{ x, z, y }`
- `findSupportY`: 指定した `{ x, z }` の床高さを返す関数
- `isHorizontalBlocked`: 水平方向の移動が壁や柱に当たるかを返す関数
- `robotRadius`: ロボット半径
- `options`: 段差高さ、探索範囲、グリッド幅、安全余白など

出力:

- `ok`: 経路生成に成功したか
- `path`: 追従に使う3D折れ線経路 `[{ x, z, y }, ...]`
- `polylineMetrics`: 経路長などの距離情報
- `debugData`: `3D Route Search` に渡す探索可視化データ
- `visitedCount`: 探索したノード数
- `reason`: 失敗理由

同一フロア、階段、複雑な3D形状で探索モードを分けているため、今後はTheta*、HPA*、ナビメッシュ系の planner と比較しやすい構成にできます。

### 経路追従モジュールの考え方

Go2 と TurtleBot3 では移動機構が違うため、同じ経路でも追従方針を変えています。

- Go2: waypoint を順に踏む、直線移動とその場 yaw 回転を分ける
- TurtleBot3: 曲線経路を滑らかに追従する

今後は、Pure Pursuit、Stanley Control、MPC などを別 follower として追加し、同じマップ、同じ障害物、同じ開始点と目的地で比較する構成にできます。

## アセット出典

### Unitree Go2

- 製品ページ: https://www.unitree.com/go2/
- Unitree ROS: https://github.com/unitreerobotics/unitree_ros
- Unitree model: https://github.com/unitreerobotics/unitree_model
- 出典メモ: `web/assets/go2/ATTRIBUTION.md`

### TurtleBot3 Burger

- ROBOTIS TurtleBot3: https://github.com/ROBOTIS-GIT/turtlebot3
- 参考 URDF: `turtlebot3_description/urdf/turtlebot3_burger.urdf`
- 出典メモ: `web/assets/turtlebot3/ATTRIBUTION.md`

## 注意

- これは物理シミュレータではなく、ブラウザベースの可視化・検証用ビューアーです。
- Go2 の歩行表現や TurtleBot3 の見た目は、操作感と検証用途を優先して調整しています。
- 3D Tiles 上の当たり判定は、読み込まれたタイルのメッシュに対する簡易判定です。
- 3D Tiles 上の経路計画は検証用の簡易実装で、階段や同一フロアでは高速化のために近似判定を使います。
- PLATEAU の大規模データは重いため、ブラウザやPC性能によって読み込みに時間がかかる場合があります。
- Viewer、Editor、3D Tiles Viewer は、用途ごとに状態を分けながら段階的に整理しています。

## 旧スクリプト

旧 2D SVG マップ生成スクリプトも残しています。

```bash
python scripts/create_go2_map.py
```

出力:

- `output/go2_map.svg`
