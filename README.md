# robot-nav-viewer

脚型ロボットと車輪型ロボットの両方を扱える、ブラウザベースの兼ナビゲーション検証用3D ビューアです。

現在は主に次の 2 機体を対象にしています。

- `Unitree Go2`
- `TurtleBot3 Burger`

主な機能:

- 3D マップ表示
- 手動操作
- 経路生成
- 自律移動
- 3D マップ編集

## スクリーンショット

<img width="1917" height="910" alt="viewer" src="https://github.com/user-attachments/assets/2161b38f-4cd0-40aa-b2bc-83e43c8245d4" />
<img width="1910" height="898" alt="planner" src="https://github.com/user-attachments/assets/5af936f0-4beb-439d-a598-8ff2a44fc174" />
<img width="1916" height="903" alt="editor" src="https://github.com/user-attachments/assets/7ed41186-1846-4f51-9ce8-0d98e62a1aa7" />

## できること

- ブラウザ上で障害物つき 3D マップを表示
- `Go2` と `TurtleBot3 Burger` を切り替えて表示
- 手動でロボットを操作
- 障害物を避ける経路を生成
- 経路生成過程を 2D デバッグパネルで可視化
- 別画面の `Map Editor` で障害物やスポーン位置を編集
- 障害物を 3D 上で移動・拡大縮小

## ロボットごとの挙動

### Unitree Go2

- 関節つきの 3D 四足モデル
- 胴体座標系ベースの手動移動
- 折れ線経路ベースの自律追従
- 経路上の最近点から自然に合流して追従開始

### TurtleBot3 Burger

- STL メッシュを使った車輪型モデル
- 曲線経路ベースの自律追従
- 車輪回転アニメーション

注意:

- TurtleBot3 の見た目位置合わせはまだ調整中です。車体、車輪、センサの位置関係に微調整が残っている可能性があります。

## 画面構成

### Viewer

- ファイル: `web/index.html`
- 用途: ロボット操作、経路生成、自律移動、デバッグ表示

### Editor

- ファイル: `web/editor.html`
- 用途: 3D マップ編集専用画面

## 主なファイル

- `web/index.html`
  ビューア画面の UI と HUD
- `web/editor.html`
  マップ編集画面
- `web/viewer.js`
  Three.js 本体、ロボット描画、入力、経路生成、エディタ操作
- `web/assets/go2/`
  Go2 用アセットと出典
- `web/assets/turtlebot3/`
  TurtleBot3 用アセットと出典
- `config/go2_map.json`
  初期マップ設定
- `scripts/create_go2_map.py`
  旧 2D SVG マップ生成スクリプト

## 起動方法

リポジトリのルートでローカル HTTP サーバを起動します。

```bash
python -m http.server 8000
```

Viewer:

```text
http://localhost:8000/web/
```

Editor:

```text
http://localhost:8000/web/editor.html
```

## 必要なもの

- Python 3
- WebGL 対応ブラウザ
  - Google Chrome
  - Microsoft Edge
  - Firefox
  - Safari
- CDN 読み込み用のインターネット接続
  - `three`
  - `three/addons`

## Viewer の操作

### ロボット操作

- `W`: 前進
- `A`: 左移動
- `S`: 後退
- `D`: 右移動
- `← / →`: yaw 回転
- `F`: スプリント
- `Shift`: スニーク
- `Space`: ジャンプ

### カメラ

- `Camera` ボタンで追従視点 / 自由視点を切り替え
- 自由視点では:
  - `LMB`: orbit
  - `RMB`: pan
  - `Wheel`: zoom

### UI

- `E`: 操作ガイドの表示 / 非表示

## 経路生成と自律移動

1. `Start` で始点を置く
2. `Goal` で終点を置く
3. `Plan` で障害物回避つき経路を生成
4. `Auto` で自律追従を開始

経路表示の考え方:

- `Go2` は折れ線経路を使って追従
- `TurtleBot3` は平滑化した曲線経路を使って追従

## Editor の操作

- `LMB`: オブジェクト選択
- `LMB drag`: 選択した障害物を移動
- `空き地で LMB drag`: カメラ orbit
- `RMB drag`: カメラ pan
- `Wheel`: zoom

選択した障害物には 3D ハンドルが表示されます。

- 水色: 幅
- オレンジ: 奥行
- ピンク: 高さ

## Editor のツール構成

### Session

- `Guide`
- `Close`

### Box Editing

- `Select`
- `Add Obstacle`
- `Delete Obstacle`

### Robot / Spawn

- `Set Spawn`

右側パネルでは次を編集できます。

- マップの幅 / 高さ
- 追加障害物のデフォルトサイズ
- 選択中障害物の位置 / サイズ / elevation
- ロボットの spawn yaw

## アセット出典

### Unitree Go2

- 製品ページ: https://www.unitree.com/go2/
- ROS 記述: https://github.com/unitreerobotics/unitree_ros
- モデル配布: https://github.com/unitreerobotics/unitree_model
- 出典メモ: `web/assets/go2/ATTRIBUTION.md`

### TurtleBot3 Burger

- リポジトリ: https://github.com/ROBOTIS-GIT/turtlebot3
- 参照 URDF: `turtlebot3_description/urdf/turtlebot3_burger.urdf`
- 出典メモ: `web/assets/turtlebot3/ATTRIBUTION.md`

## 旧スクリプト

旧 2D SVG マップ生成スクリプトも残しています。

```bash
python scripts/create_go2_map.py
```

出力:

- `output/go2_map.svg`

参照設定:

- `config/go2_map.json`

## 補足

- これは物理シミュレータではなく、ブラウザベースの可視化 / デモ環境です
- 自律移動や歩容は、見た目と操作感を重視して調整しています
- Viewer と Editor はブラウザストレージ経由でマップ状態を共有しています
- 特に TurtleBot3 の STL 見た目合わせは、まだ改善の余地があります

## 今後の改善候補

- TurtleBot3 の見た目位置合わせの改善
- より本格的な 3D リサイズ gizmo
- マップの import / export
- viewer / planner / editor のモジュール分割
- ROS 連携の強化
