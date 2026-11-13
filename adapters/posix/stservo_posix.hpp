/** @file stservo_posix.hpp @brief Raspberry Pi・USB-UART向けのLinux termios通信アダプター。 */
#pragma once
#include "STServo/stservo.hpp"
#include <termios.h>

namespace stservo {
/** @brief ノンブロッキング・8N1のシリアルポートのファイル記述子を所有します。
 * @details OSのバッファを使う全二重UARTアダプターです。単線通信の送受信方向は
 * 外部のサーボインターフェースで切り替えてください。PiのGPIOを直接駆動する実装ではありません。
 * UART・USBデバイスの選択とハードウェア設定はアプリケーション側で行います。
 * 終了時に元の端末設定を復元します。スレッドセーフではありません。
 */
class PosixTransport final : public Transport {
public:
    /** @brief ポートを閉じた状態で生成します。 */
    PosixTransport();
    /** @brief 元の端末設定を復元し、ファイル記述子を解放します。 */
    ~PosixTransport() override;
    PosixTransport(const PosixTransport&) = delete;
    PosixTransport& operator=(const PosixTransport&) = delete;
    /** @brief 既存のデバイス（例：/dev/ttyAMA0）を開き、raw・8N1に設定します。
     * @return 失敗時はfalse。errorNumber()でerrnoの値を取得できます。
     * @note 既に開いている場合はEBUSYで失敗し、既存のポートを維持します。
     */
    bool open(const char* device, std::uint32_t baud = 1000000);
    /** @brief 所有するファイル記述子を閉じます。繰り返し呼び出しても安全です。 */
    void close();
    /** @brief 現在シリアルポートを開いているかどうかを返します。 */
    bool isOpen() const;
    /** @brief 最後に記録したシステムエラー（errnoの値）を返します。 */
    int errorNumber() const;
    bool write(const std::uint8_t* data, std::size_t size, std::uint32_t timeout_ms) override;
    int readByte() override;
    bool discardInput() override;
    std::uint32_t nowMs() const override;
    void idle() override;
private:
    int fd_;
    int error_;
    termios original_;
    bool original_valid_;
};
}
