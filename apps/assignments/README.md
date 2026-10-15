# 課題フォルダ

課題のプログラムは、このフォルダ以下に一つずつ保存します。元のサンプル（`apps/`直下）は変更しません。

## サンプルから課題を作る

1. [Actions → Create assignment from sample](https://github.com/temesotejam/spike-rt-web-toolkit/actions/workflows/create-assignment.yml) を開き「Run workflow」を押します。
2. `sample_id` にサンプルのフォルダ名（例 `pd_multi_task1`）、`assignment_id` に課題ID（例 `kadai_01`）、`title` に表示名を指定します。
3. 完了すると `apps/assignments/kadai_01/kadai_01.c` と、元サンプルにあった必要な設定ファイルができます。
4. 課題の `.c`（必要に応じて `.cfg` など）を編集・コミットします。通常のアプリと同じように自動ビルドされ、Web画面の「課題」グループから選択できます。

課題IDには半角英字で始まる英数字・アンダースコアを使用してください。既存のアプリIDとの重複は不可です。

