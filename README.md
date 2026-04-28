# Codex_start

Unitree Go2 の実験環境づくりの第一歩として、**WebGL (ブラウザ) で2Dマップ上のGo2初期配置を確認できるビューワ**を追加しました。

## どの環境で実行できる？

### 1) ブラウザ（推奨）
- Google Chrome / Microsoft Edge / Firefox / Safari
- WebGL対応GPUがあれば動作
- 依存ライブラリ（Three.js）はCDNから読み込み

### 2) 既存のSVG生成スクリプト（CLI）
- Python 3 があれば実行可能
- 静的な `output/go2_map.svg` を出力

## ブラウザ版の起動

リポジトリ直下で以下を実行し、ブラウザで `http://localhost:8000/web/` を開いてください。

```bash
python3 -m http.server 8000
```

## ブラウザ版の構成

- `web/index.html`: ビューワ本体
- `web/viewer.js`: Three.jsベースのWebGL描画
- `config/go2_map.json`: Go2の初期座標・向きとマップサイズ

`config/go2_map.json` を編集すれば、ブラウザ表示にも反映されます（リロード時）。


## GitHubへ反映する手順

GitHubリポジトリURLが分かっている場合、以下のスクリプトで `origin` 設定と push をまとめて実行できます。

```bash
./scripts/push_to_github.sh <github_repo_url> [branch]
# 例
./scripts/push_to_github.sh git@github.com:your-name/your-repo.git work
```

## CLI版（静的SVG）の実行

```bash
python3 scripts/create_go2_map.py
```

実行すると以下が生成されます。
- 設定ファイル: `config/go2_map.json`（存在しなければ自動生成）
- 画像: `output/go2_map.svg`

## 次ステップ候補

- ROS 2 + Gazebo上でURDF/SDFモデルのGo2をスポーン
- 2D SLAM/Navigation用のoccupancy mapへ接続
- LiDAR/IMUの疑似センサ入力を追加
