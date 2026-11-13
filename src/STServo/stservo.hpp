/**
 * @file stservo.hpp
 * @brief STSサーボ向けのプラットフォーム非依存FT-SCSバスクライアント（C++11）。
 */
#pragma once
#include <cstddef>
#include <cstdint>

namespace stservo {
/** @brief 移植用の抽象インターフェース。1つの通信アダプターを1つのバスが専有します。
 * @details 新しい環境では5つの操作を実装してください。write() に渡されたバッファを
 * 呼び出し終了後まで保持してはいけません。readByte() は速やかに戻る必要があります。
 * ボーレート、8N1、半二重通信の送受信方向切り替えは共通処理の外で設定します。
 */
class Transport {
public:
    /** @brief 抽象インターフェース経由で通信アダプターを破棄します。 */
    virtual ~Transport() = default;
    /** @brief 時間内にフレーム全体を受け付けます。失敗時はfalseを返します。
     * @param data フレームのバッファ。この呼び出しが戻るまでのみ有効です。
     * @param size バイト数。
     * @param timeout_ms 一連の通信処理に残された制限時間（ミリ秒）。
     * @return 全バイトを受け付けた場合のみtrue。サーボのACK受信を意味しません。
     * @note OSの送信バッファを使う場合、戻った後も物理的な送信が続くことがあります。
     * 方向を手動で切り替える半二重ポートでは、実装側で送信完了を待ち、受信へ切り替えてください。
     */
    virtual bool write(const std::uint8_t* data, std::size_t size,
                       std::uint32_t timeout_ms) = 0;
    /** @return 0～255：受信バイト、-1：受信データなし、-2以下：通信アダプターのエラー。 */
    virtual int readByte() = 0;
    /** @brief 新しい入力を待たずに、受信済みのデータを破棄します。 */
    virtual bool discardInput() = 0;
    /** @brief 待機中も進む単調増加時刻（ミリ秒）。2の32乗で周回します。 */
    virtual std::uint32_t nowMs() const = 0;
    /** @brief 短時間の待機または実行権の譲渡を行います。制限時間内に速やかに戻ってください。 */
    virtual void idle() = 0;
};
} // namespace stservo

/** @brief 複数IDを扱うシリアルバス。HAL依存やヒープ確保はありません。
 * @details 各操作は同期的に実行します。物理バスごとに1つのインスタンスを共有し、
 * 異なるスレッドからの呼び出しも含め、呼び出し元で排他制御してください。
 * 通信アダプターはこのオブジェクトより長く存続する必要があります。IDの事前登録は不要です。
 */
class STServo {
public:
    static constexpr std::uint8_t MaxId = 253; ///< 個別通信に使える最大ID。
    static constexpr std::uint8_t BroadcastId = 254; ///< ブロードキャスト先ID。個別のACKはありません。
    static constexpr std::size_t MaxPacketSize = 64; ///< ヘッダーを含む通信フレームの最大バイト数。
    static constexpr std::size_t MaxReadSize = MaxPacketSize - 6; ///< READで取得できるデータの最大バイト数。
    static constexpr std::size_t MaxWriteSize = MaxPacketSize - 7; ///< WRITEで書き込めるデータの最大バイト数。
    static constexpr std::size_t MaxSyncVelocityServos = (MaxPacketSize - 8) / 3; ///< 1回のSYNC WRITEで2バイトの速度を指定できる最大台数。

    /** @brief サーボのERRORバイトとは別の、フレーム処理・通信アダプター側の結果。 */
    enum class Result : std::uint8_t {
        OK = 0x00, CHECKSUM_ERROR = 0x10, HEADER_ERROR = 0x20,
        ID_ERROR = 0x22, LENGTH_ERROR = 0x23, TIMEOUT = 0x24,
        BUFFER_ERROR = 0x25, INVALID_ARGUMENT = 0x26,
        TRANSPORT_ERROR = 0x27, ECHO_ONLY = 0x28, ECHO_ERROR = 0x29
    };
    /** @brief 実際の電気的インターフェースに合わせて指定します。自動判定はしません。
     * @warning PINGのエコーは有効なERROR=1の応答と同じバイト列です。LocalEchoでは、
     * 実際のステータスパケットの前に完全なローカルエコーが1つだけ返る必要があります。
     */
    enum class EchoMode { None, LocalEcho };
    /** @brief WRITEでステータスパケットを待つかどうか。サーボの応答設定に合わせてください。 */
    enum class WriteResponse { None, Required };

    /** @brief 要求したIDに対応する結果。直前に操作した別のサーボの結果は含みません。
     * @note OKはローカルの通信処理の成功を示します。サーボから正常応答を受信したと判断するには、
     * さらに status_received && servo_error == 0 を確認してください。
     */
    struct Reply {
        Result result; ///< ローカルのプロトコル処理・通信アダプターの結果。
        std::uint8_t id; ///< 要求した宛先ID。
        std::uint8_t servo_error; ///< サーボのERRORバイト。status_receivedがtrueの場合のみ有効です。
        bool status_received; ///< ステータスフレームの検証に成功した場合のみtrue。
        std::size_t parameter_count; ///< 受信したパラメーターのバイト数。
        /** @brief 指定した宛先について、応答未受信のローカル結果を生成します。 */
        Reply(Result result_, std::uint8_t id_);
        bool ok() const; ///< ローカルの通信処理が成功したかどうかのみを返します。
    };

    /** @param transport このバスが専有するシリアル通信・時刻アダプター。
     * @param timeout_ms 操作全体の制限時間（1～0x7fffffffミリ秒）。
     * @param echo ステータス応答の前にローカルエコーを読み捨てるかどうか。
     */
    explicit STServo(stservo::Transport& transport, std::uint32_t timeout_ms = 10,
                     EchoMode echo = EchoMode::None);
    STServo(const STServo&) = delete;
    STServo& operator=(const STServo&) = delete;

    /** @brief 1つのID（0～253）へ問い合わせます。ブロードキャストPINGは受け付けません。
     * @param id 個別通信の宛先ID。
     * @return 通信結果と、受信できた場合は指定IDのサーボステータス。
     */
    Reply ping(std::uint8_t id);
    /** @brief 連続するレジスタを読み出します（1～MaxReadSizeバイト）。
     * @param id 個別通信の宛先ID。
     * @param address 先頭レジスタのアドレス。
     * @param size 読み出すデータのバイト数。
     * @param output 長さが正しく有効なERROR=0の応答を受信した場合のみ更新します。
     * @note サーボのERRORが0以外の場合も応答受信済みのReplyを返しますが、
     * outputは更新しません。エラー応答ではパラメーターが省略される場合があります。
     */
    Reply readRegister(std::uint8_t id, std::uint8_t address,
                       std::uint8_t* output, std::size_t size);
    /** @brief 連続するレジスタへ書き込みます。ブロードキャストはACK不要の場合のみ使用できます。
     * @param id 個別通信の宛先ID。ACK不要の場合はBroadcastIdも使用できます。
     * @param address 先頭レジスタのアドレス。
     * @param data 連続するレジスタへ書き込む値。
     * @param size 書き込むデータのバイト数。
     * @param response ステータス応答を待つかどうか。
     * @note response=Noneの場合、OKは送信が受け付けられたことのみを示し、サーボ側の処理成功は保証しません。
     */
    Reply writeRegister(std::uint8_t id, std::uint8_t address,
                        const std::uint8_t* data, std::size_t size,
                        WriteResponse response = WriteResponse::None);
    /** @brief 1回の0x83ブロードキャストで、複数IDの同じレジスタ範囲へ書き込みます。
     * @param address 全サーボ共通の先頭レジスタアドレス。
     * @param bytes_per_servo IDごとのレジスタデータのバイト数。
     * @param ids 重複のない0～253のID配列。
     * @param data idsの順に各サーボのデータを並べた配列。長さはcount * bytes_per_servoバイトです。
     * @param count サーボ台数。フレーム全体がMaxPacketSize以内に収まる必要があります。
     * @return ローカルの送信結果。SYNC WRITEには個別の応答がありません。
     */
    Result syncWriteRegister(std::uint8_t address, std::uint8_t bytes_per_servo,
                             const std::uint8_t* ids, const std::uint8_t* data,
                             std::size_t count);
    /** @brief トルク出力（0x28）をONまたはOFFにします。
     * @param id 宛先ID。ACK不要の場合はブロードキャストも使用できます。
     * @param enabled trueでON、falseでOFF。位置・速度指令は変更しません。
     * @param response サーボの応答設定に合わせたACK方針。
     * @note ONにすると既存の目標値に従って動作します。先に目標値を設定してください。
     */
    Reply setTorqueEnabled(std::uint8_t id, bool enabled,
                           WriteResponse response = WriteResponse::None);
    /** @brief STSの位置制御モード（0x21 = 0）へ切り替えます。
     * @param id 宛先ID。ACK不要の場合はブロードキャストも使用できます。
     * @param response サーボの応答設定に合わせたACK方針。
     * @note トルクOFF、EEPROMロック操作、目標設定は呼び出し元で行ってください。
     * 角度制限・分解能は変更しません。保存にはEEPROMの解除・再ロックが必要です。
     */
    Reply setPositionMode(std::uint8_t id, WriteResponse response = WriteResponse::None);
    /** @brief 単回転の目標位置をレジスタ0x2a/0x2bへ書き込みます。
     * @param id 宛先ID。ACK不要の場合はブロードキャストも使用できます。
     * @param position 目標位置（0～4095）。範囲外は送信せずINVALID_ARGUMENTを返します。
     * @param response サーボの応答設定に合わせたACK方針。
     * @return 通信結果。正常ACKは目標への到達を意味しません。
     * @note 位置モードとトルクONを別途設定してください。移動速度はsetVelocity()で指定します。
     * 通常の分解能1では2048が約180°です。角度制限・分解能は変更しません。
     * 多回転・符号付き目標にはwriteRegister()を使用してください。
     */
    Reply setPosition(std::uint8_t id, std::uint16_t position,
                      WriteResponse response = WriteResponse::None);
    /** @brief IDレジスタ（0x05）を書き換えます。ACKを待たず、新IDで読み戻して確認してください。
     * @param id 現在の個別ID（0～253）。ブロードキャストは拒否します。
     * @param new_id 変更先の個別ID（0～253）。使用中のIDは指定しないでください。
     * @return ローカルの送信結果。OKのみでは変更成功・保存成功を保証しません。
     * @note 対象を1台だけ接続し、トルクOFF、旧IDでEEPROM解除、setId()、
     * 新IDで0x05を読み戻し、新IDでEEPROM再ロックの順に操作してください。
     * 応答のIDが変更前か変更後かに依存しないよう、WRITEのACKは待ちません。
     */
    Reply setId(std::uint8_t id, std::uint8_t new_id);
    /** @brief EEPROM保存ロック（0x37）を設定します。
     * @param id 宛先ID。ACK不要の場合はブロードキャストも使用できます。
     * @param locked falseで解除（EEPROM書込を永続化）、trueで再ロック。
     * @param response サーボの応答設定に合わせたACK方針。
     * @note この関数自体は他の設定を変更せず、トルクも切り替えません。
     */
    Reply setEepromLocked(std::uint8_t id, bool locked,
                          WriteResponse response = WriteResponse::None);
    /** @brief STSの動作モードレジスタ0x21へ1を書き込みます（連続回転・速度モード）。
     * @param id 宛先ID。ACK不要の場合はブロードキャストも使用できます。
     * @param response ステータス応答を待つかどうか。
     */
    Reply setVelocityMode(std::uint8_t id, WriteResponse response = WriteResponse::None);
    /** @brief STSの目標速度レジスタ0x2e/0x2fへ書き込みます。絶対値は32767に制限します。
     * @param id 宛先ID。ACK不要の場合はブロードキャストも使用できます。
     * @param velocity サーボの機種固有のレジスタ単位で指定する、符号付き目標速度。
     * @param response ステータス応答を待つかどうか。
     * @details 負数の場合は方向ビット（BIT15）を立てます。バイト順はリトルエンディアンです。
     * 使用する機種のレジスタ定義を確認してください。
     */
    Reply setVelocity(std::uint8_t id, std::int32_t velocity,
                      WriteResponse response = WriteResponse::None);
    /** @brief 旧関数名を残したsetVelocity()の別名。IDの明示指定が必要です。
     * @param id 宛先ID。
     * @param velocity 符号付き目標速度。旧形式の1引数呼び出しとの混同を防ぐため省略できません。
     */
    Reply sts_exchange(std::uint8_t id, std::int32_t velocity);
    /** @brief 1回のSYNC WRITEフレームで、各IDへ個別の符号付き速度を送信します。
     * @param ids 重複のない宛先ID配列。
     * @param velocities ID配列と同じ順序の符号付き速度配列。
     * @param count ID・速度の組数。最大MaxSyncVelocityServos組です。
     * @note 各サーボの動作モードは別途設定してください。
     */
    Result syncSetVelocity(const std::uint8_t* ids, const std::int32_t* velocities,
                           std::size_t count);
    /** @brief 操作全体のタイムアウトを変更します。falseの場合は変更しません。
     * @param timeout_ms 制限時間（ミリ秒）。1～0x7fffffffの範囲で指定します。
     */
    bool setTimeoutMs(std::uint32_t timeout_ms);
    /** @brief 設定済みの操作全体のタイムアウトを返します。 */
    std::uint32_t timeoutMs() const;
    /** @brief 物理インターフェースの既知のローカルエコー動作を指定します。
     * @param mode 既知のハードウェア動作。PINGのバイト列から推測してはいけません。
     */
    void setEchoMode(EchoMode mode);
    /** @brief 設定済みのローカルエコー動作を返します。 */
    EchoMode echoMode() const;
    /** @brief ヘッダーを含む完全なフレームのチェックサムを設定します（size >= 6）。
     * @param frame チェックサム領域を含む、書き込み可能なフレーム。
     * @param size 2バイトのヘッダーを含むフレーム全体のバイト数。
     */
    static void genCheckSum(std::uint8_t* frame, std::size_t size);
    /** @brief IDからチェックサムまでのパケットを検証します（先頭のFF 2バイトは含めません）。
     * @param packet IDで始まりチェックサムで終わるバイト列。
     * @param size ヘッダーを除くパケットのバイト数。4以上を指定します。
     */
    static std::uint8_t receiveCheckSum(const std::uint8_t* packet, std::size_t size);
    /** @brief 表示用の静的な結果名を返します。文字列の所有権はライブラリにあります。 */
    static const char* resultName(Result result);

private:
    enum class Command : std::uint8_t { Ping = 1, Read = 2, Write = 3, SyncWrite = 0x83 };
    Reply exchange(std::uint8_t id, const std::uint8_t* frame, std::size_t size,
                   bool expect_status, std::size_t expected_parameters,
                   std::uint8_t* output);
    static void encodeVelocity(std::int32_t velocity, std::uint8_t* output);
    stservo::Transport& transport_;
    std::uint32_t timeout_ms_;
    EchoMode echo_;
};
