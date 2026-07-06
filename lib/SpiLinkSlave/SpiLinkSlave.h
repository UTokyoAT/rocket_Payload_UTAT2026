#pragma once
#include <stdint.h>
#include "spi_protocol.h"

// XIAO2側（スレーブ）。XIAO1（マスター）からのトランザクションを受け身で待つ。
// hideakitai/ESP32SPISlave の queue/trigger/wait 方式を使用。
// NOTE: ESP32SPISlave.h はコールバック関数をヘッダ内に非inlineで定義しているため、
//       複数の.cppからincludeすると多重定義でリンクエラーになる。
//       ここでは前方宣言のみとし、実体（new ESP32SPISlave()）はSpiLinkSlave.cppに閉じ込める。
//       `ESP32SPISlave` はクラスではなく `arduino::esp32::spi::slave::Slave` へのusingエイリアス
//       なので、前方宣言は実体のクラスに対して行う必要がある。
namespace arduino { namespace esp32 { namespace spi { namespace slave {
class Slave;
}}}}

class SpiLinkSlave {
public:
    SpiLinkSlave();
    ~SpiLinkSlave();

    void begin();

    // 新しいフレームを受信していればtrueを返しoutに格納する。
    // 呼び出すたびに次のトランザクション用のキューを積み直す。
    bool poll(SpiFrameToXiao2& out);

    // 次にマスターへ返す応答フレームを設定する（次回のpoll()呼び出しまでに反映）
    void setResponse(const SpiFrameFromXiao2& in);

private:
    arduino::esp32::spi::slave::Slave* _slave;
    uint8_t _txBuf[SPI_FRAME_SIZE] = {0};
    uint8_t _rxBuf[SPI_FRAME_SIZE] = {0};
};
