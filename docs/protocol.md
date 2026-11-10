# プロトコルと制約

## フレーム

命令は `FF FF ID LENGTH INSTRUCTION PARAMETERS... CHECKSUM`、
応答は `FF FF ID LENGTH ERROR PARAMETERS... CHECKSUM` です。
`LENGTH = パラメーター数 + 2`、チェックサムはヘッダーを除くIDから最後のパラメーターまでの
8bit加算結果を反転します。[FEETECHのプロトコル定義](https://ftservo.github.io/ftservo-wiki/reference/protocol/ft-scs/)

```text
ID1 PING:       FF FF 01 02 01 FB
ID1 ERROR=0:    FF FF 01 02 00 FC
ID2 PING:       FF FF 02 02 01 FA
ID2 ERROR=0:    FF FF 02 02 00 FB
```

PING・READ・ACK付きWRITEは、完全な応答のIDとチェックサムを検査します。
別IDの完全フレームは読み飛ばして宛先IDの応答を待ち、ID違いだけならTIMEOUTになります。
破損フレームの後に正常なフレームが来れば復帰します。途中でタイムアウトしたフレームは次の操作に持ち越しません。
毎回の入力破棄は遅れて到着した旧応答をすべて識別できる保証ではありません。
このプロトコルにシーケンス番号はないため、タイムアウト後の同一ID/同一長の遅延応答は区別不能です。

## 複数IDとSync Write

0〜253を通常ID、254をブロードキャストとして扱います。
複数台からの応答衝突を避けるためブロードキャストPING/READ/ACK付きWRITEはAPIで拒否します。
ACKなしWRITEとSYNC WRITEは254で送信でき、個々の応答を待ちません。
SYNC WRITEは `0x83` と、レジスタ先頭・1台分サイズ・`ID + データ` の繰り返しを送ります。
重複IDや不正IDは送信前に拒否します。サーボ設定や個別受理を確認する場合は後からREAD等を行ってください。

## 固定長バッファ

| 項目 | 上限 |
|---|---:|
| コアのフレーム全長 | 64バイト |
| READのデータ | 58バイト |
| WRITEのデータ | 57バイト |
| Sync Write台数 | `floor(56 / (1台分データサイズ + 1))` |
| `syncSetVelocity` | 18台/フレーム |

超過はBUFFER_ERRORで送信しません。連続レジスタ範囲が0xffを越える要求はINVALID_ARGUMENTです。
ID宛ての順次通信に18台という制約はありません。これは速度Sync Writeの1フレームの上限です。
大きなグループを分割した場合、グループ間は同時更新ではなくなります。

## 時刻と結果

1〜0x7fffffffミリ秒を許可し、uint32_tの差分でTick周回を処理します。
無受信はTIMEOUT、受信フレームの破損はCHECKSUM_ERROR/LENGTH_ERROR、
I/O失敗はTRANSPORT_ERRORです。複数の破損フレームだけで終わった場合は最後の解析エラーを返します。
`servo_error` は機種依存のビットフィールドなので、コアでVoltage等の文字列へ決め打ち変換しません。

## STSの便利関数

`setVelocityMode` は0x21へ1を、`setVelocity` は0x2e/0x2fへ速度を送ります。
速度の絶対値は32767で飽和し、負方向をbit15で表して下位バイトから送信します。
これらは既存のSTS実装の機種依存部分です。SCSCL等の別系列では共通のPING/READ/WRITEを利用し、
各モデルのレジスタ表・エンディアン・速度形式に従ってください。
