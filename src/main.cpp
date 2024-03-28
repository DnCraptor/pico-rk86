#include <cstdlib>
#include <cstring>
#include <hardware/clocks.h>
#include <hardware/flash.h>
#include <hardware/structs/vreg_and_chip_reset.h>
#include <pico/bootrom.h>
#include <pico/time.h>
#include <pico/multicore.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>
#include "graphics.h"
#include "psram_spi.h"

extern "C" {
#include "ps2.h"
#include "nespad.h"
#include "ff.h"
#include "debug.h"
#include "util_Wii_Joy.h"

#include "i8080_hal.h"
#include "i8080.h"
#include "vg75.h"
#include "vv55_i.h"
#include "keymap.h"
#include "tape.h"
#include "timer0.h"
#include "ui.h"
#include "ps2_rk.h"
#include "ps2_codes_rk.h"
#include "main.h"
#include "menu.h"
#include "tv.h"
}

static FATFS fs;
semaphore vga_start_semaphore;

pwm_config config = pwm_get_default_config();
void PWM_init_pin(uint8_t pinN, uint16_t max_lvl) {
    gpio_set_function(pinN, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&config, 1.0);
    pwm_config_set_wrap(&config, max_lvl); // MAX PWM value
    pwm_init(pwm_gpio_to_slice_num(pinN), &config, true);
}

void inInit(uint gpio) {
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

extern "C" uint8_t RAM[0x8000];

void __time_critical_func(render_core)() {
    const auto buffer = RAM;
    graphics_set_buffer(buffer, 320, 240); // ??
    graphics_set_textbuffer(buffer);
    multicore_lockout_victim_init();
    graphics_init();
    graphics_set_textbuffer(buffer);
    graphics_set_bgcolor(0x000000);
    graphics_set_offset(0, 0);
    graphics_set_flashmode(false, false);
    sem_acquire_blocking(&vga_start_semaphore);
    // 60 FPS loop
#define frame_tick (16666)
    uint64_t tick = time_us_64();
#ifdef TFT
    uint64_t last_renderer_tick = tick;
#endif
    uint64_t last_input_tick = tick;
    while (true) {
#ifdef TFT
        if (tick >= last_renderer_tick + frame_tick) {
            refresh_lcd();
            last_renderer_tick = tick;
        }
#endif
        // Every 5th frame
        if (tick >= last_input_tick + frame_tick * 5) {
            nespad_read();
            last_input_tick = tick;
            //nespad_update();
        }
        tick = time_us_64();
        tight_loop_contents();
    }
    __unreachable();
}

#ifdef SOUND
static repeating_timer_t timer;
static int snd_channels = 2;
static bool __not_in_flash_func(snd_timer_callback)(repeating_timer_t *rt) {
    static uint16_t outL = 0;  
    static uint16_t outR = 0;
    register size_t idx = sound_array_idx;
    if (idx >= sound_array_fill) {
        return true;
    }
    pwm_set_gpio_level(PWM_PIN0, outR); // Право
    pwm_set_gpio_level(PWM_PIN1, outL); // Лево
    outL = outR = 0;
    if (!Sound_enabled || paused) {
        return true;
    }
    register UBYTE* uba = LIBATARI800_Sound_array;
    if (snd_channels == 2) {
        outL = uba[idx++]; idx++;
        outR = uba[idx++]; idx++;
    } else {
        outL = outR = uba[idx++]; idx++;
    }
    sound_array_idx = idx;
    ///pwm_set_gpio_level(BEEPER_PIN, 0);
    return true;
}
#endif

#include "f_util.h"
static FATFS fatfs;
bool SD_CARD_AVAILABLE = false;
static void init_fs() {
    FRESULT result = f_mount(&fatfs, "", 1);
    if (FR_OK != result) {
        printf("Unable to mount SD-card: %s (%d)", FRESULT_str(result), result);
    } else {
        SD_CARD_AVAILABLE = true;
    }
}

inline static void init_wii() {
    if (Init_Wii_Joystick()) {
        Wii_decode_joy();
        printf("Found WII joystick");
    }
}

#include <map>
static std::map<uint16_t, uint16_t> char2rk;
extern "C" void rk_2_at(uint16_t rk, uint16_t at) {
    char2rk[at] = rk;
}
extern "C" uint16_t rk_by_at(uint16_t at) {
    std::map<uint16_t, uint16_t>::const_iterator i = char2rk.find(at);
    if (i == char2rk.end()) return 0;
    uint16_t res = i->second;
    printf("%04Xh: %04Xh", at, res);
    return res;
}

static const uint32_t freq = 366 * KHZ;
static float i8080_takts_in_ms = 1.98;

int main() {
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    sleep_ms(10);
    set_sys_clock_khz(freq, true);
    stdio_init_all();
    keyboard_init();
    keyboard_send(0xFF);
    nespad_begin(clock_get_hz(clk_sys) / 1000, NES_GPIO_CLK, NES_GPIO_DATA, NES_GPIO_LAT);

    nespad_read();
    sleep_ms(50);

    // F12 Boot to USB FIRMWARE UPDATE mode
    if (nespad_state & DPAD_START /*|| input_map.keycode == 0x58*/) { // F12
        printf("reset_usb_boot");
        reset_usb_boot(0, 0);
    }

    init_fs(); // TODO: psram replacement (pagefile)
    init_psram();

    sem_init(&vga_start_semaphore, 0, 1);
    multicore_launch_core1(render_core);
    sem_release(&vga_start_semaphore);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < 6; i++) {
        pwm_set_gpio_level(BEEPER_PIN, 255);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        pwm_set_gpio_level(BEEPER_PIN, 0);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    PWM_init_pin(BEEPER_PIN, (1 << 8) - 1);
#ifdef SOUND
    PWM_init_pin(PWM_PIN0, (1 << 8) - 1);
    PWM_init_pin(PWM_PIN1, (1 << 8) - 1);
#endif
#if LOAD_WAV_PIO
    //пин ввода звука
    inInit(LOAD_WAV_PIO);
#endif

#ifdef SOUND
    int hz = 44100;
    // negative timeout means exact delay (rather than delay between callbacks)
    if (!add_repeating_timer_us(-1000000 / hz, snd_timer_callback, NULL, &timer)) {
        printf("Failed to add timer");
    }
#endif

    // Инитим процессор
    i8080_hal_init();
    i8080_init();
    i8080_jump(0xF800);

    tv_init();
    vg75_init((uint8_t*)i8080_hal_memory());

    // Инитим клавиатуру
    kbd_init();
    keymap_init();

    // Инитим магнитофон
    tape_init();
    // Запускаем эмуляцию
    uint32_t prev_T = getCycleCount();
    uint32_t sec_T = prev_T;
    uint32_t sec_cycles = 0;
    bool win = false;
    graphics_set_textbuffer(screen.vram);
    graphics_set_mode(TEXTMODE_DEFAULT);
    uint16_t pc = 0;
    while(true) {
        uint32_t T;
        int32_t dT = 0;
        // Можно запускать эмуляцию проца
   	    uint16_t takts = i8080_instruction();
        i8080_cycles += takts;
        do {
            T = getCycleCount();
            dT = T - prev_T;
        } while(i8080_takts_in_ms && dT < takts / i8080_takts_in_ms);
        prev_T = T;
        sec_cycles += takts;
        if ( (T - sec_T) >= 1000000) {
            // Прошла секунда
            if (i8080_takts_in_ms != 1.98)
                snprintf((char*)screen.vram, 64, "%d %d*%d %f", sec_cycles, screen.screen_w, screen.screen_h, i8080_takts_in_ms);
            //kbd_dump();
            sec_cycles = 0;
            sec_T = T;
        }
	    // Вся периодика
	    if (tape_periodic()) {
	        // Закончена запись на магнитофон - надо предложить сохранить файл
	        ui_start();
		    tape_save();
	        ui_stop();
    	    // Сбрасываем время циклов
	        sec_T = prev_T = getCycleCount();
	        sec_cycles = 0;
	    }
        if (win) {
	        // Win нажата - обрабатываем спец-команды
	        uint16_t c = ps2_read();
            if (c > 0) pc = c;
	        switch (c) {
                case 0:
                    break;
		        case PS2_LEFT:
		            // Экран влево
		            if (screen.x_offset > 0) screen.x_offset--;
		            break;
		        case PS2_RIGHT:
		            // Экран вправо
		            if (screen.x_offset < 16) screen.x_offset++;
		            break;
		        case PS2_UP:
		            // Экран вверх
		            if (screen.y_offset > 8) screen.y_offset -= 8;
                    else screen.y_offset = 0;
		            break;
		        case PS2_DOWN:
		            // Экран вниз
		            if (screen.y_offset < 8 * 8) screen.y_offset += 8;
		            break;
		        case PS2_L_WIN | 0x8000:
		        case PS2_R_WIN | 0x8000:
		            // Отжали Win
		            win = false;
		            break;
        		default:
		            printf("Unprocessed scancode (in win): %04Xh", c);
		            break;
	        }
	    } else {
	        // Win не нажата
	        uint16_t c;
	        bool rst = false;
    	    ps2_leds(kbd_rus(), true, !i8080_takts_in_ms);
    	    c = keymap_periodic();
            if (c > 0) pc = c;
    	    switch (c) {
    		    case 0:
    		        break;
    			case PS2_ESC:
    		        // Меню
		            ui_start();
			        menu();
		            ui_stop();
		            rst = true;
		            break;
				case PS2_F5:
		            // Переход на ПЗУ
		            i8080_jump(0xE000);
		            break;
				case PS2_F6:
		            // Переход на ПЗУ
		            i8080_jump(0xE004);
		            break;
				case PS2_F7:
		            // Переход на ПЗУ
		            i8080_jump(0xE008);
		            break;
				case PS2_F8:
		            // Переход на ПЗУ
		            i8080_jump(0xE00C);
		            break;
		    	case PS2_F10:
		            // Переход на ПЗУ
		            i8080_jump(0xE010);
		            break;
				case PS2_F9:
		            // Переход на ПЗУ
		            i8080_jump(0xE014);
		            break;
				case PS2_F11:
		            // Выход в монитор
        		    i8080_jump(0xF800);
		            break;
				case PS2_F12:
		            // Файловый менеджен
		            ui_start();
			        menu_fileman();
		            ui_stop();
		            rst = true;
		            break;
				case PS2_PAUSE:
		            // Сброс
        	        i8080_init();
        	        i8080_hal_init();
        	        i8080_jump(0xF800);
        	        break;
				case PS2_PRINT:
                    change_color();
		            break;
				case PS2_SCROLL:
		            // Переключатель турбо
		            i8080_takts_in_ms += 0.33;
                    if (i8080_takts_in_ms < 0) i8080_takts_in_ms = 3.30;
		            break;
				case PS2_L_WIN:
		        case PS2_R_WIN:
		            // Нажали Win
		            win = true;
		            break;
				case PS2_MENU:
		            // Отобразить справку
	        	/// TODO:    help_display();
		            break;
        		default:
		            printf("Unprocessed scancode: %04Xh", c);
		            break;
    	    }
    	    if (rst) {
	            // Сбрасываем время циклов
	        	sec_T = prev_T = getCycleCount();
	        	sec_cycles = 0;
	        }
    	}
        tight_loop_contents();
    }
    __unreachable();
}
