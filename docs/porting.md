# 移植インターフェース

## 呼び出し関係

```text
アプリケーション
  STServo（1バス、複数ID、フレーム処理）
    stservo::Transport（純粋仮想）
      STM32Transport → STM32BufferedSerial / HAL
      PosixTransport → termios / poll / steady_clock
      独自Transport → 新しいMCUやOSのAPI
```

共通の `src/STServo/stservo.hpp` と `src/stservo.cpp` はISO C++11でコンパイルできます。
新しいプラットフォームは `Transport` の5関数を実装し、初期化した実装を参照で渡します。
`Transport` のデストラクターはvirtualです。バスより長く生存させてください。

| 純粋仮想関数 | 契約 |
|---|---|
| `write(data, size, timeout_ms)` | 期限内に全バイトを受け付けたらtrue。部分送信や失敗はfalse。ポインターを保持しない |
| `readByte()` | 非ブロッキング。0〜255は受信値、-1は未受信、-2以下はI/O失敗 |
| `discardInput()` | すでに受信済みのデータだけを破棄し、成功ならtrue。新規受信を待たない |
| `nowMs() const` | uint32_tの単調ミリ秒時刻。待機中にも進み、周回してよい |
| `idle()` | 短い待機・yield。処理を独占せず、期限を大きく超えて待たない |

`write()` のtrueは、サーボ受理・正常状態の確認ではありません。
Linuxでは全バイトをOSへ渡した時点で戻り、物理送信が続いている場合があります。
手動方向切替の半二重ポートは、派生クラス側で送信完了と受信への切替を完了させてください。
現在のLinuxアダプターは方向自動切替の外付けバス変換基板を想定しています。

## 新しい移植層の骨格

```cpp
class MyTransport final : public stservo::Transport {
public:
    bool write(const std::uint8_t* data, std::size_t size,
               std::uint32_t timeout_ms) override;
    int readByte() override;
    bool discardInput() override;
    std::uint32_t nowMs() const override;
    void idle() override;
};
```

GPIO、UARTの8N1・ボーレート、割り込み、方向制御の初期化はアプリケーション/移植層の責務です。
共通ライブラリはピン、デバイス名、UART番号を選びません。
タイムアウトは入力破棄・送信・応答待ちを含む1操作全体に適用します。
最初の受信バイトからタイマーを延長する方式ではないため、途中フレームでも待機が伸び続けません。
呼び出し元でバス全体を排他制御し、ISRから同期APIを呼ばないでください。

## STM32実装

ワークスペースの `stservo/include/stm32_stservo_transport.hpp` と
`stservo/src/stm32_stservo_transport.cpp` が実装例です。

```cpp
MX_Init();
STM32BufferedSerial uart(&huart1);
uart.begin();
stservo::STM32Transport transport(uart);
STServo bus(transport, 100);
auto reply = bus.ping(1);
```

UARTの送信はHALのブロッキング送信、受信は既存の割り込みバッファを利用します。
同じUARTで `uart.write()` を混用したり、別の送信元を動かしたりしないでください。
NVIC、IRQハンドラーからのHAL呼び出し、HAL完了/エラーコールバックの接続が必要です。
既存の `lib_archive = no` は維持してください。mxpio syncはこのアプリケーション部分を上書きしません。

## 所有権・メモリ・同時実行

- 1物理バスに1つの `STServo` を共有します。IDごとにバスオブジェクトを作る必要はありません。
- 各操作は `Reply` を値で返します。ID1の結果を保存してからID2へ通信しても、保存値は変わりません。
- コアは固定長バッファを使い、ヒープ・例外・RTOSに依存しません。移植層やCLIは別です。
- バスオブジェクトのコピーは禁止しています。複数スレッドで使う場合はmutex等で各API呼び出し全体を排他制御します。
- 同じバスへ複数のマスターを接続する構成は対象外です。
