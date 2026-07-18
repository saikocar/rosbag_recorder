# rosbag_recorder CPUバースト対策の経緯と今後の改修計画

作成: 2026-07-17

## 背景

rosbag_recorder（実運用は `rosbag_recorder_video2ch`）は60秒ごとに `ros2 bag record` プロセスを
SIGINT停止→再起動する方式でローテーションしており、毎分CPUバーストが発生していた
（対策前: ピーク約100%、対策後: 約66%）。

## 実施済みの対策 (2026-07-17)

1. **mcapストレージ化 + キャッシュ拡大**
   `rosbag_recorder.py` / `rosbag_recorder_video2ch.py` の記録コマンドに追加:
   `--storage mcap --max-cache-size 268435456` (256MiB, デフォルト100MiB)
   - mcapはsqlite3より書き込み負荷が軽い
   - 注意: bagを別PCで解析する場合は `ros-humble-rosbag2-storage-mcap` が必要

2. **config.yamlのtypo修正で動画録画を無効化**
   `vdeo_enabled: false` → `video_enabled: false`（typoのためキーが読まれず、
   2026-07-14に無効化したつもりのffmpeg×2本(1920x1080 MJPEG→NVENC)が動き続けていた）。
   毎分のCUDA/NVENCセッション再初期化×2と常時エンコード負荷が消え、これがピーク低減の主因。

## 残っている毎分バーストの正体（計測済み）

2026-07-17に pidstat/mpstat で140秒(2ローテーション)計測した結果:

- ローテーション時刻(毎分:19秒)に新しい `ros2 bag record` (Python) が **約64%×1秒×1コア** を消費。
  内訳はPythonインタープリタ起動 + rclpy/rosbag2のimport + 約100トピックのsubscription確立。
- 32コアマシンで全体CPUは平坦(約970%/3200%)。停車中は周囲のAutowareノードへの実害なし。
- ただし**走行中・交通量の多い区間ではベース負荷が上がるため、このスパイクは無視できない**。
  → 下記の改修を後日実施する。

## 今後の改修計画: 内部split化（プロセス再起動の廃止）

### 方針

`ros2 bag record` を起動しっぱなしにし、`--max-bag-duration 60` によるrosbag2内部の
ファイルsplitでローテーションを代替する。毎分のプロセス再起動（＝バースト源）が完全に消える。

### 現行の挙動で維持すべきもの

- **不要セグメントの破棄**: 現在は「セグメント中にshould_record（disengage/MRM/メモ/lv4区間）が
  立たなかったbagディレクトリ」を1周期遅れで削除。イベント直前の1分は文脈として残る仕様。
- **in_segment（lv4区間 / lanelet区間）の1本化**: 区間中はローテーションをスキップして
  1本の長いbagにしている（`ROTATE_SKIP_FAILSAFE_LIMIT = 30` のフェイルセーフ付き）。
- **memo.txt**: bagディレクトリ単位でメモを書き出し。
- **journal退避**: 保存対象bagのみ `_save_journal_async` 発火（video2ch側は現在コメントアウト中）。

### 設計スケッチ

1. ノード起動時に1回だけ `ros2 bag record -o <run_dir> --storage mcap --max-cache-size 268435456
   --max-bag-duration 60 <topics...>` を起動。以後プロセスは維持。
2. 出力は `<run_dir>/<basename>_0.mcap, _1.mcap, ...` と連番で生える。
   ファイルNは「ファイルN+1が出現した時点」でクローズ済みとみなせる。
3. 60秒タイマーは維持し、役割を「プロセス再起動」から「クローズ済みsplitファイルのGC判定」に変更:
   - 各splitに対し、その期間内に should_record が立ったかを記録しておく
   - 「自分も次のsplitも未マーク」のファイルを削除（現行の1周期遅れ削除と同じ意味論）
4. in_segment中はGCを止めるだけでよい（splitは60秒ごとに切れるが全ファイル保持されるので
   内容としては現行の「1本の長いbag」と等価。再生はmcapなら個別ファイル再生可）。
5. memo.txt は run_dir に集約し、タイムスタンプで対応付け（現行もメモ行にdatetimeが付いている）。
6. **metadata.yamlの扱いに注意**: metadata.yamlは録画終了時にしか書かれず、splitファイルを
   削除すると不整合になる。mcapは自己完結形式なので個別ファイルの `ros2 bag play <file>.mcap` は
   可能。ディレクトリ単位の再生が必要なら削除後に `ros2 bag reindex <run_dir> mcap` で再生成する。
7. ディスクフル対策: プロセスが長寿命になるため、run_dirの日次切り替え（1日1回だけ再起動を許容）
   などの上限を設ける。

### 留意点

- 記録の取捨選択（イベント時の保全）は安全上重要なロジックなので、改修後は
  「未マーク区間が消えること」「disengage/MRM時に該当split＋直前splitが残ること」を
  実車データで必ず確認する。
- `--max-bag-duration` によるsplitタイミングとタイマーのGC判定の境界ズレ（±数秒）を許容する
  設計にする（splitファイルのmtimeで期間を判定するのが確実）。

## 改修実施計画（次の非運行日に実施。例: 2026-07-20(月)）

土日が運行日のため、改修とテストは非運行日に行い、次の運行日までに実車相当の動作確認を終えること。

### 事前条件

- 車両が運行中でないこと
- tmuxの単体起動が残っていれば停止: `tmux kill-session -t rosbag_recorder_node`
- 作業前に現行コードの動作するコミットを確認しておく（切り戻し先）

### 実装ステップ

1. **configフラグで新旧を切り替え可能にする**（切り戻しを設定1行にするため）
   - `config.yaml` に `use_internal_split: false` を追加（デフォルトは現行動作）
   - `rosbag_recorder_video2ch.py` に新方式の分岐を実装。現行のrotate方式のコードは削除しない
2. **新方式の実装**（上記「設計スケッチ」参照）
   - `start_bag()`: `--max-bag-duration 60` 付きで1回だけ起動、以後再起動しない
   - タイマーコールバック: プロセス再起動の代わりに、クローズ済みsplitファイル
     （次の連番ファイルが出現したもの）へのマーク判定とGC（未マーク×2連続で削除）
   - splitごとの should_record 記録: タイマー周期ごとに「現在openなsplitファイル名→マーク有無」を
     dictで保持。境界ズレはファイルmtimeで吸収
   - in_segment中はGCを停止（ファイルは切れるが全部残す）
   - memo.txt は run_dir 直下に集約
3. **run_dirの日次切り替え**: 1日1回（例: 起動から24時間経過後の非記録タイミング）だけ
   プロセスを再起動してrun_dirを切り替える

### 机上テスト（実車不要。月曜のうちに完了させる）

1. ダミーtopicをpubしながらノード起動: `ros2 topic pub -r 10 /autoware/state ...` 等、
   または実機のAutowareを起動せず適当なtopic数本だけconfigに入れた縮小構成で確認
2. **GC動作**: 何もマークしない状態で3〜4分放置 → 古いsplitが2周期遅れで消えること
3. **イベント保全**: `/record_memo_text` にpub → 該当splitと直前のsplitが残ること
   （現行の「イベント直前1分を残す」意味論と一致すること）
4. **in_segment**: lanelet_info をpubして区間に入れ、区間中のsplitが全て残ること・
   フェイルセーフ（30回スキップ）が効くこと
5. **再生確認**: 残ったsplitを個別に `ros2 bag play <file>.mcap` できること、
   `ros2 bag reindex <run_dir> mcap` でmetadata.yamlを再生成できること
6. **異常系**: recordプロセスをkill -9して、ノードが検知/復帰（または少なくともエラーログ）すること

### 運行日前の最終確認（金曜または運行日朝）

- `use_internal_split: true` で本番構成のconfig（全topic）にて15分程度記録し、
  CPUバーストが毎分出ないこと（`pidstat 1` / mpstatで確認）
- disengage相当のイベントを1回発生させ、bag・memo.txt・(有効なら)journal退避が揃うこと
- 問題があれば `use_internal_split: false` に戻すだけで現行動作に復帰できることを確認

### 切り戻し

- 第一手段: `config.yaml` の `use_internal_split: false`（コード変更不要、再起動のみ）
- 最終手段: 本ドキュメント作成時点のコミットへgit revert

## 運用メモ

- 2026-07-17時点、レコーダーはAutoware launchから切り離してtmuxセッション
  `rosbag_recorder_node` で単体起動中（ログ: `~/rosbag_recorder_node.log`）。
  **次回Autoware全体をrelaunchする前に `tmux kill-session -t rosbag_recorder_node` を実行すること**
  （launch側 `launch_rosbag_recorder:=true` からも起動されて二重記録になるため）。
