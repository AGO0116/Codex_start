# Codex_start

Three.js ベースの 3D ロボットビューアです。  
Unitree Go2 を中心に、TurtleBot3 Burger も切り替えて表示でき、手動操作と簡易な自律移動デモを試せます。

## Features

- 3D マップ表示
- `Unitree Go2` と `TurtleBot3 Burger` の切り替え
- Go2 の簡易歩容アニメーション
- TurtleBot3 の差動二輪っぽい走行表現
- `WASD + Left / Right + Space` ベースの手動操作
- スタート / ゴール指定、経路計画、経路可視化、自律移動
- 障害物回避、段差・階段の簡易判定

## Project Layout

- `web/index.html`
  3D ビューアのエントリポイントと HUD
- `web/viewer.js`
  描画、入力、ロボット切り替え、経路計画、自律移動の本体
- `web/assets/go2/`
  Go2 のメッシュと出典メモ
- `web/assets/turtlebot3/`
  TurtleBot3 Burger のメッシュと出典メモ
- `config/go2_map.json`
  旧 2D マップ生成スクリプト用の設定
- `scripts/create_go2_map.py`
  SVG の 2D マップ画像を生成する補助スクリプト

## Run

ビルドは不要です。ローカル HTTP サーバを立てて `web/` を開いてください。

```bash
python -m http.server 8000
```

ブラウザで次を開きます。

```text
http://localhost:8000/web/
```

## Requirements

- Python 3
- WebGL 対応ブラウザ
  - Google Chrome
  - Microsoft Edge
  - Firefox
  - Safari
- インターネット接続
  - `three` と `three/addons` を CDN から読み込みます

`requirements.txt` に追加依存がないのは、このリポジトリ自体には Python パッケージ依存がないためです。

## Controls

基本操作:

- `W / A / S / D`: ロボット座標系で前後左右移動
- `Left / Right`: yaw 回転
- `Ctrl`: スプリント
- `Shift`: スニーク
- `Space`: ジャンプ
- マウスドラッグ: 視点周回
- マウスホイール: ズーム

補足:

- Go2 は四足ロボット向けの移動表現です
- TurtleBot3 は差動二輪らしく、横移動を使わない前提の挙動に寄せています

## Auto Navigation

HUD の `Auto Nav` セクションから操作できます。

1. `Set Start` を押して始点をクリック
2. `Set Goal` を押して終点をクリック
3. `Plan Path` で経路生成
4. `Start Auto` で自律移動開始

現在の経路計画は、長方形障害物をもとにした 2D の可視グラフと最短経路探索をベースにしています。  
生成した経路は画面上に可視化され、ロボットはその経路に沿ってゆっくり追従します。

## Robot Assets

### Unitree Go2

- 公式ページ  
  https://www.unitree.com/go2/
- 公式 ROS モデル  
  https://github.com/unitreerobotics/unitree_ros
- モデル配布リポジトリ  
  https://github.com/unitreerobotics/unitree_model

このリポジトリでは `web/assets/go2/ATTRIBUTION.md` に出典メモがあります。

### TurtleBot3 Burger

- 公式リポジトリ  
  https://github.com/ROBOTIS-GIT/turtlebot3
- 参照 URDF  
  `turtlebot3_description/urdf/turtlebot3_burger.urdf`

このリポジトリでは `web/assets/turtlebot3/ATTRIBUTION.md` に出典メモがあります。

## Legacy 2D Script

初期段階で使っていた 2D SVG 生成スクリプトも残しています。

```bash
python scripts/create_go2_map.py
```

出力先:

- `output/go2_map.svg`

設定:

- `config/go2_map.json`

## Notes

- このビューアは物理シミュレータではありません
- ロボット運動は見た目とデモ用途を重視した簡易実装です
- 自律移動も研究用途の厳密な制御器ではなく、デモ用の追従ロジックです
- TurtleBot3 の STL 姿勢合わせは引き続き調整中です

## Future Ideas

- URDF ベースのより厳密なリンク / 関節再現
- ロボットごとの運動モデル切り替え強化
- より自然な経路平滑化と追従制御
- マップ編集 UI
- ROS 連携やセンサ可視化
