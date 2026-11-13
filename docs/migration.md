# 旧APIからの移行

2.0は意図的なAPI変更です。古いヘッダーの呼び出しを黙って別の意味で動かす互換層は設けていません。
`stservo/include/stservo.hpp` は共通ヘッダーへの薄い転送用として残しています。

| 旧 | 新 |
|---|---|
| `STServo st(&usb, &uart, 1, 300)` | `STM32Transport port(uart); STServo bus(port, 300);` |
| `st.ping()` | `auto reply = bus.ping(1);` |
| `uint8_t result` | `STServo::Reply`。`reply.result` が `enum class Result` |
| `st.getServoError()` | その操作の `reply.servo_error`（`status_received` を先に確認） |
| `st.writeRegister(addr, data, len)` | `bus.writeRegister(id, addr, data, len, policy)` |
| `st.setVelocityMode()` | `bus.setVelocityMode(id)` |
| `st.sts_exchange(500)` | `bus.setVelocity(id, 500)` または必ず2引数の `bus.sts_exchange(id, 500)` |
| `st.receive()` | 同期操作が解析まで担当。移植層は `Transport::readByte()` を実装 |
| `getLastPacketLength()` | `Reply::parameter_count`。応答全長が必要なら `parameter_count + 6` |

USBログ用ポインターは通信ライブラリへ渡しません。ログはアプリケーション側で出します。
STM32に加えてRaspberry Piでは `stservo::PosixTransport` を同じコンストラクターへ渡せます。
PlatformIOには `symlink://../shared/STServo` を追加済みです。

## 正常確認

```cpp
auto reply = bus.ping(id);
if (reply.result == STServo::Result::OK && reply.status_received) {
    if (reply.servo_error == 0) { /* 正常応答 */ }
    else { /* サーボが報告したエラー。ローカル通信失敗とは区別する */ }
}
```

ACKなしWRITEは `result == OK` でも `status_received == false` です。
旧コードは送信待ちバッファへ入れた時点でOKを返していました。新APIも、ACKを要求しない場合は
サーボの受理確認を意味しません。送信が途中で失敗した場合は `TRANSPORT_ERROR` にします。

現在の `stservo/src/main.cpp` はID1・ID2へのPINGとIDレジスタ読み取りの例です。
旧サンプルの動作モード書き込みや速度指令は自動実行しません。
既存APIのSTS速度レジスタ/符号形式は維持していますが、実際の動作は使用機種の制御表と照合してください。

旧 `sts_exchange(速度)` が新APIでID指定と解釈されないよう、別名の `sts_exchange` もID・速度の2引数を必須にしています。速度モードの停止はPhase（0x12）BIT3=0を確認した上で `setVelocity(id, 0)` を使います。BIT3=1では速度0が最高速度を意味するため、設定を確認してください。
