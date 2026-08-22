# SPIKE-RT Web Toolkit

SPIKE-RTアプリを`github.dev`で編集し、GitHub Actionsでビルドし、GitHub PagesからSPIKE Prime HubへWebUSB DFU書き込みし、実行中のUSBシリアルログをWeb Serialで確認するためのWebツールキットです。

このリポジトリは、`temesotejam/spike-rt-web-project` の実機確認済みコミット `599e7eaffca891e422489b78fb8e3ad903e2d2ac` を基準にしています。その構成では、GitHub Actionsで生成したSPIKE-RT v0.2.0の`asp.bin`をWebUSBで実機へ書き込み、DFU終了・再起動後にLED countdownが実行されるところまでEnd-to-Endで確認済みです。

## 現在の機能

- `github.dev`でCソースを編集
- GitHub ActionsでSPIKE-RT v0.2.0を固定ビルド
- `apps/`以下の複数アプリを一括ビルド
- GitHub Pagesからアプリを選択
- SHA-256確認
- WebUSB / DfuSeでSPIKE Prime Hubへ書き込み
- SPIKE-RTは`0x08008000`へ書き込み
- DFU終了時は公式`pydfu.py`と同様に`0x08000000`をブート先として指定
- 書き込みごとにUSB転送サイズとDFU状態を確認
- Windows初回設定用の専用WinUSBセットアップをPagesから案内
- Web SerialでSPIKE-RTのUSBシリアルログをブラウザ表示
- Web Serialは115200 bps
- Web Serialのポート候補はVID/PIDで絞り込まず、ブラウザのシリアルポート選択画面からユーザーが選択

## Windows 初回設定

WindowsでDFUモードのHubをWebUSBから選択できない場合は、GitHub Pagesの「Windows 初回設定」から専用セットアップを取得します。

配布する`SPIKE-RT-DFU-WinUSB-Setup-v0.3.exe`は単一EXEのWindows x64アプリです。対象をLEGO SPIKE Prime / Technic Large HubのDFUモード`VID 0x0694 / PID 0x0008`に限定し、SPIKE-RT実行中のUSBシリアル / COMポートや他のUSB機器は変更しません。

```text
Pagesの「Windows 初回設定」を開く
→ WinUSBセットアップをダウンロード
→ HubをDFUモードでUSB接続
→ SPIKE-RT-DFU-WinUSB-Setup-v0.3.exeを起動
→ 「準備完了」を確認
→ Pagesへ戻ってWebUSB書き込み
```

すでに`Service=WinUSB`の場合は何も変更せず「準備完了」と表示します。実機ではDFU Hubの検出、LEGOメタデータ確認、設定済みWinUSB判定、設定済み時の無変更動作、非破壊INF生成まで確認済みです。WinUSB未設定PCでの新規ドライバ割り当て経路のみ最終実機確認待ちです。

セットアップ本体は`temesotejam/spike-rt-dfu-winusb-setup`のGitHub Releaseで管理し、このToolkitにはEXEのコピーを置きません。PagesはRelease Assetへのダウンロード導線だけを提供します。

## Web Serialデバッグ

SPIKE-RT v0.2.0公式ドキュメントでは、実行中のHubのログをUSBシリアル`/dev/ttyACM0`へ115200 bpsで接続して確認します。このリポジトリでは同じログをChrome / EdgeのWeb Serial APIでブラウザ内に表示します。

Windowsでは、必要に応じてデバイスマネージャーの「ポート (COMとLPT)」でSPIKE-RTが使用しているCOM番号を確認します。Pagesの「デバッグ接続」を押すと、Chrome / Edgeが利用可能なシリアルポートを表示するので、そのCOMポートを選択します。

Web側ではVID/PIDによるフィルタや接続拒否を行いません。USB記述子の違いに依存せず、選択されたポートを115200 bpsで開きます。`getInfo()`で取得できるVID/PIDは、接続後に参考情報として表示します。

使い方は次の流れです。

```text
WebUSBでasp.binを書き込む
→ HubがDFUを終了してSPIKE-RTを起動
→ 必要ならデバイスマネージャーでCOM番号を確認
→ Pagesの「デバッグ接続」を押す
→ SPIKE-RTのCOMポートを選ぶ
→ syslog()の出力をブラウザで確認
```

`button`アプリはWeb Serialの動作確認に向いています。起動時に`BUTTON`を出力し、その後は左右・中央・Bluetoothボタンを押すたびにログを出力します。

Web Serial側はDFU用WinUSBとは別です。通常起動したSPIKE-RTのUSBシリアルドライバをWinUSBへ変更しないでください。WinUSBはDFUモード`VID 0x0694 / PID 0x0008`のWebUSB書き込み用として扱います。

## 今後の拡張

1. UI簡略化・診断機能
2. Web Serialログの変数表示・グラフ化

## 収録アプリ

- `myapp`: 最小構成
- `button`: Hubボタン入力をUSBシリアルログへ出力
- `led`: a〜zを5×5表示へ表示
- `led_fast`: 0〜9を0.25秒ごとに表示
- `led_countdown`: 9〜0を1秒ごとに表示
- `motor`: ポートAのモーターを回転・停止

## 基本フロー

```text
github.devでapps/<app>/<app>.cを編集
→ Commit & Push
→ GitHub Actionsでビルド
→ GitHub Pagesへasp.binを公開
→ 必要ならWindows初回WinUSB設定
→ SPIKE Prime HubをDFUモードにする
→ PagesからHubへ接続
→ 書き込み
→ DFU終了・再起動
→ SPIKE-RTアプリを自動実行
→ Web Serialでログ確認
```

## WebUSB対象

- LEGO SPIKE Prime Hub DFUモード
- VID: `0x0694`
- PID: `0x0008`
- SPIKE-RT load address: `0x08008000`
- boot address: `0x08000000`
- 最大サイズ: 992 KiB

WindowsでWebUSB利用のためにDFUデバイスへWinUSBを割り当てる必要がある場合は、Pagesの「Windows 初回設定」から専用セットアップを利用します。

## GitHub Pages

`Settings` → `Pages` → `Build and deployment` → `Source` を `GitHub Actions` に設定します。

## ライセンス

このリポジトリのライセンスは`LICENSE`、第三者コードについては`THIRD_PARTY_NOTICES.md`を参照してください。
