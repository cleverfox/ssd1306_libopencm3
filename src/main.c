/*
 * Copyright (c) 2017, Stanislav Lakhtin. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include <ssd1306_i2c.h>

static void clock_setup(void) {
  rcc_clock_setup_in_hse_12mhz_out_72mhz();

  /* Enable GPIOs clock. */
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);

  /* set clock for I2C */
  rcc_periph_clock_enable(RCC_I2C2);

  /* set clock for AFIO*/
  rcc_periph_clock_enable(RCC_AFIO);

  AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;
}

static void i2c_setup(void) {
  /* Set alternate functions for the SCL and SDA pins of I2C2. */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                GPIO_I2C2_SCL | GPIO_I2C2_SDA);

  /* Disable the I2C before changing any configuration. */
  i2c_peripheral_disable(I2C2);

  /* APB1 is running at 36MHz. */
  i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_36MHZ);

  /* 400KHz - I2C Fast Mode */
  i2c_set_fast_mode(I2C2);

  /*
	 * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
	 * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
	 * Datasheet suggests 0x1e.
	 */
  i2c_set_ccr(I2C2, 0x1e);

  /*
   * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
   * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
   * Incremented by 1 -> 11.
   */
  i2c_set_trise(I2C2, 0x0b);

  /*
   * Enable ACK on I2C
   */
  i2c_enable_ack(I2C2);

  /*
   * This is our slave address - needed only if we want to receive from
   * other masters.
   */
  i2c_set_own_7bit_slave_address(I2C2, 0x32);

  /* If everything is configured -> enable the peripheral. */
  i2c_peripheral_enable(I2C2);
}

volatile int8_t step = 0;

void exti9_5_isr(void) {
  if (!gpio_get(GPIOA, GPIO8) && gpio_get(GPIOA, GPIO9))
      step += 1;
  else
      step -= 1;
  exti_reset_request(EXTI8); // we should clear flag manually
}

static void board_setup(void) {
  // Debug setting for rotary encoder EC11 on (PA8, PA9) for make simple command
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO8 | GPIO9);

  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
  exti_enable_request(EXTI8);
  exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
  exti_select_source(EXTI8, GPIOA);
}

int main(void) {
  /**
   * Brief delay to give the debugger a chance to stop the core before we
   * muck around with the chip's configuration.
   */
  for (uint32_t loop = 0; loop < 1000000; ++loop) {
    __asm__("nop");
  }

  clock_setup();
  i2c_setup();
  board_setup();

  struct lcd mylcd;
  ssd1306_init(&mylcd, I2C2, DEFAULT_7bit_OLED_SLAVE_ADDRESS, 128, 32);
  ssd1306_send_data(&mylcd, COMMAND, 0xc8); //updown flip
  ssd1306_send_data(&mylcd, COMMAND, 0xA1); //mirror horizonally
  /* //interlace mode for some 128x64 disaplys
  uint8_t rlcdcfg[]={0xda,0x10};
  ssd1306_command(&rlcd, rlcdcfg, 2);
   */

  step = 1;
  int16_t y = 0;

  while (1) {
      if (step!=0) {
        for (int i =0; i<8; i++) {
          y += step;
          ssd1306_clear(&mylcd);
          ssd1306_drawWCharStr(&mylcd, 0, y, white, wrapDisplay, L"Привет! Это длинный текст c цифрой 01234567890 и " \
          "cимволами .!№;%:?*()), специально, чтобы создать проблемы для отрисовки.\n"  \
          "There is a lot text with ENGLISH or latin symbols!\n\n"\
          "Однажды в далёкой-далёкой галактике LOREM IPSUM\n\n "
              //         "\"Sed ut perspiciatis unde omnis iste natus error sit voluptatem accusantium doloremque laudantium, totam rem aperiam, eaque ipsa quae ab illo inventore veritatis et quasi architecto beatae vitae dicta sunt explicabo. Nemo enim ipsam voluptatem quia voluptas sit aspernatur aut odit aut fugit, sed quia consequuntur magni dolores eos qui ratione voluptatem sequi nesciunt. Neque porro quisquam est, qui dolorem ipsum quia dolor sit amet, consectetur, adipisci velit, sed quia non numquam eius modi tempora incidunt ut labore et dolore magnam aliquam quaerat voluptatem. Ut enim ad minima veniam, quis nostrum exercitationem ullam corporis suscipit laboriosam, nisi ut aliquid ex ea commodi consequatur? Quis autem vel eum iure reprehenderit qui in ea voluptate velit esse quam nihil molestiae consequatur, vel illum qui dolorem eum fugiat quo voluptas nulla pariatur?\""
          );
          ssd1306_refresh(&mylcd);
          for (uint32_t loop = 0; loop < 1000000; ++loop) {
            __asm__("nop");
        }
      }
      if (step<0)
        step += 1;
      else
        step -= 1;
    }
  }

  /* We will never get here */
  return 0;
}
