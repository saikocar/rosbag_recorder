# rosbag_recorder

本パッケージは、トピックの内容や制御信号に基づいて自動的にrosbagの記録を開始・停止し、期間ごとの記録とメモ情報を保存します。

## 特徴

- 制御トピックに応じたrosbag記録の自動開始/停止
- トピックリストを設定ファイルで柔軟に管理
- メモトピックを受信して記録ディレクトリに `memo.txt` を保存
- 記録ファイルは日時で階層化されたディレクトリに保存
- Python製ノードでROS2 launchからの起動に対応

## 構成

```
rosbag_recorder/
├── config.yaml
├── rosbag_recorder.py
├── index.html
├── recorder_ui.sh
├── README.md
└── package.xml
```

## 依存

- ROS 2 Humble または互換バージョン
- rclpy
- std_msgs
- ros-humble-rosbridge-server(UI用)

## 実行方法

本パッケージをクローンして利用してください。

メインノードの実行
```bash
source <your_autoware_ws>/install/setup.bash
python3 rosbag_recorder.py
```

ユーザーインターフェースの立ち上げ
```bash
~/rosbag_recorder
bash recorder_ui.sh
```

## 使い方

### 1. configの編集

`config.yaml` に記録対象のトピック、保存ルートディレクトリ、記録周期（秒）などを記述します。

```yaml
record_topics: #このセクションに記録対象のトピックを列挙する
  - /topic1
  - /topic2
......
control_topic: /autoware/state # autowareが自動走行を行っているかどうかを判別するためのトピック
memo_topic: /record_memo_text #記録事由を受け取るトピック
interval_sec: 60  # 記録周期（秒）
bag_output_dir: /home/sit/rosbag_recorder/  # 保存ルートディレクトリ
```

### 2. ノードの起動

```bash
source <your_autoware_ws>/install/setup.bash
python3 rosbag_recorder.py
```

### 3. UIによる記録制御

```
bash recorder_ui.sh
```

以上のスクリプトから呼び出されたブラウザ上のページからメッセージを送る`(send)`することで記録中の区間及びその一つ前の区間が保存対象になります。

```NOTE
`ros2 topic pub --once /record_memo_text std_msgs/msg/String "{data: 'ここに残したいメッセージ'}"`を実行しても同様の動作する。
```

### 4. ディレクトリ構成

記録は以下のような階層で保存されます：

```
<bag_output_dir>/YYYYMMDD/MMDDHHMM/
  ├── memo.txt       # メモ（存在する場合）
  └── rosbagに関連するデータ
```

## ライセンス

Apache 2.0 License

