#include "SPI.h"
#include "SdFatConfig.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "utility.hpp"
#include <SdFat.h>
#include <hardware/dma.h>

void spi_dma_irq_handler();

class CustomSPIClass : public SdSpiBaseClass {
 public:
  // Activate SPI hardware with correct speed and mode.
  void activate() { SPI.beginTransaction(m_spiSettings); }
  // Initialize the SPI bus.
  void begin(SdSpiConfig config) {
    (void)config;
    // csPin = config.csPin;
    // sd_spi_dma_chan = dma_claim_unused_channel(true);
    // c = dma_channel_get_default_config(sd_spi_dma_chan);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    // channel_config_set_dreq(&c, spi_get_dreq(spi0, true));
    // dma_channel_set_irq0_enabled(sd_spi_dma_chan, true);
    // irq_set_exclusive_handler(DMA_IRQ_0, spi_dma_irq_handler);
    // irq_set_enabled(DMA_IRQ_0, true);
    // SPI.begin();
  }
  // Deactivate SPI hardware.
  void deactivate() {
    // NOTE: We don't end the transaction here since the `send` call is non-blocking.
    // The transaction is ended on DMA_IRQ_0
    SPI.endTransaction();
  }
  // Receive a byte.
  uint8_t receive() { return SPI.transfer(0XFF); }
  // Receive multiple bytes.
  // Replace this function if your board has multiple byte receive.
  uint8_t receive(uint8_t* buf, size_t count) {
    SPI.transfer(nullptr, buf, count);
    // for (size_t i = 0; i < count; i++) {
    //   buf[i] = SPI.transfer(0XFF);
    // }
    return 0;
  }
  // Send a byte.
  void send(uint8_t data) { SPI.transfer(data); }
  // Send multiple bytes.
  // Replace this function if your board has multiple byte send.
  void send(const uint8_t* buf, size_t count) {
    // if (sd_spi_dma_chan != -1) {
    //   dma_channel_unclaim(sd_spi_dma_chan);
    // }

    // sd_spi_dma_chan = dma_claim_unused_channel(true);
    // dma_channel_config c = dma_channel_get_default_config(sd_spi_dma_chan);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    // channel_config_set_dreq(&c, spi_get_dreq(spi0, true));

    // if (sdCardInitialized) {
    //   Serial.printf("Called custom SPI multi-byte write with DMA\n");
    //   dma_channel_configure(sd_spi_dma_chan, &c, &spi_get_hw(spi0)->dr, buf, count, true);
    //   preventDeassert = true;
    //   dma_channel_wait_for_finish_blocking(sd_spi_dma_chan);
    // } else {
      SPI.transfer(buf, nullptr, count);
    // }


    // for (size_t i = 0; i < count; i++) {
    //   SPI.transfer(buf[i]);
    // }
  }
  // Save SPISettings for new max SCK frequency
  void setSckSpeed(uint32_t maxSck) {
    m_spiSettings = SPISettings(maxSck, MSBFIRST, SPI_MODE0);
  }

  SdCsPin_t csPin;
  bool preventDeassert = false;
 private:
  SPISettings m_spiSettings;
  dma_channel_config c;

} customSpi;

// void spi_dma_irq_handler() {
//   SPI.endTransaction();
//   digitalWrite(customSpi.csPin, HIGH);
//   customSpi.preventDeassert = false;
// }

// void sdCsWrite(SdCsPin_t pin, bool level) {
//   // Only allow this function to assert the CS, not de-assert it
//   Serial.printf("CUSTOM sdCsWrite called with pin=%d, level=%d\n", pin, level);
//   if (customSpi.preventDeassert) {
//     if (!level) {
//       digitalWrite(pin, level);
//     } else {
//       Serial.printf("\tCustom SPI multi-byte write was used, NOT de-asserting\n");
//     }
//   } else {
//     Serial.printf("Potentially allowing de-assert\n");
//     digitalWrite(pin, level);
//   }
// }
