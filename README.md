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
- Web SerialでSPIKE-RTのUSBシリアルログをブラウザ表示
- Web Serialは115200 bps、LEGO VID `0x0694`で候補を絞り込み
- DFU PID `0x0008`はデバッグ接続として拒否

## Web Serialデバッグ

SPIKE-RT v0.2.0公式ドキュメントでは、実行中のHubのログをUSBシリアル`/dev/ttyACM0`へ115200 bpsで接続して確認します。このリポジトリでは同じログをChrome / EdgeのWeb Serial APIでブラウザ内に表示します。

使い方は次の流れです。

```text
WebUSBでasp.binを書き込む
→ HubがDFUを終了してSPIKE-RTを起動
→ Pagesの「デバッグ接続」を押す
→ SPIKE Prime HubのUSBシリアルを選ぶ
→ syslog()の出力をブラウザで確認
```

`button`アプリはWeb Serialの動作確認に向いています。起動時に`BUTTON`を出力し、その後は左右・中央・Bluetoothボタンを押すたびにログを出力します。

Web Serial側はDFU用WinUSBとは別です。通常起動したSPIKE-RTのUSBシリアルドライバをWinUSBへ変更しないでください。WinUSBはDFUモード`VID 0x0694 / PID 0x0008`のWebUSB書き込み用として扱います。

## 今後の拡張

1. WinUSB専用セットアップ支援
2. UI簡略化・診断機能
3. Web Serialログの変数表示・グラフ化

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

WindowsではWebUSB利用のためDFUデバイスへWinUSBドライバを割り当てる必要がある場合があります。今後、この初期設定を安全に行う専用ツールを追加する予定です。

## GitHub Pages

`Settings` → `Pages` → `Build and deployment` → `Source` を `GitHub Actions` に設定します。

## ライセンス

このリポジトリのライセンスは`LICENSE`、第三者コードについては`THIRD_PARTY_NOTICES.md`を参照してください。
