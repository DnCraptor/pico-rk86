#include "ps2.h"

///#include "gpio_lib.h"
#include "timer0.h"
#include "pt.h"
#include "board.h"


//http://www.avrfreaks.net/sites/default/files/PS2%20Keyboard.pdf


#define RXQ_SIZE	16


static uint32_t prev_T=0;
static uint16_t rxq[RXQ_SIZE];
static uint8_t rxq_head=0, rxq_tail=0;

static uint16_t tx=0, txbit=0;
static uint8_t led_status;
static bool ack=0, resend=0, bat=0;

static struct pt pt_task;


static void gpio_int(void *arg)
{
    static uint16_t rx=0, rxbit=1;
    static bool was_E0=0, was_E1=0, was_F0=0;
    
    uint32_t gpio_status;
    gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);
    
    // Получаем время от прошлого бита, если оно слишком большое - сбросим автомат приема
    uint32_t T=getCycleCount();
    uint32_t dT=T-prev_T;
    if (dT < 8000) return;	// если фронт короче 50мкс (по стандарту 60-100мкс), то это иголка (пропускаем ее)
    if (dT > 19200)	// 120мкс таймаут
    {
	// Сбрасываем приемник
	rx=0;
	rxbit=1;
    }
    prev_T=T;
    
    if (txbit)
    {
	if (txbit & (1<<9))
	{
	    // Передали 8 бит данных и 1 бит четности - переходим в прием
	    tx=0;
	    txbit=0;
	    gpio_init_input_pu(PS2_DATA);
	} else
	{
	    // Мы в режиме передачи
	    if (tx & txbit)
		gpio_on(PS2_DATA); else
		gpio_off(PS2_DATA);
	    txbit<<=1;
	}
    } else
    {
	// Мы в режиме приема
	
	// Принимаем бит
	if (gpio_in(PS2_DATA)) rx|=rxbit;
	rxbit<<=1;
	
	// Проверяем на конец байта
	if (rxbit & (1<<11))
	{
	    // Принято 11 бит
	    if ( (!(rx & 0x001)) && (rx & 0x400) )	// проверим наличие старт и стоп битов
	    {
		// Убираем стартовый бит
		rx>>=1;
		
		// Получаем код
		uint8_t code=rx & 0xff;
		
		// Считаем четность
		rx^=rx >> 4;
		rx^=rx >> 2;
		rx^=rx >> 1;
		rx^=rx >> 8;
		
		if (! (rx & 1))
		{
		    // Все нормально !
		    if (code==0xE0) was_E0=1; else
		    if (code==0xE1) was_E1=1; else
		    if (code==0xF0) was_F0=1; else
		    if (code==0xFA) ack=1; else
		    if (code==0xFE) resend=1; else
		    if (code==0xAA) bat=1; else
		    {
			uint16_t code16=code;
			
			// Расширенные наборы
			if (was_E0) code16|=0x0100; else
			if (was_E1) code16|=0x0200;
			
			// Отжатие
			if (was_F0) code16|=0x8000;
			
			// Кладем в буфер
			rxq[rxq_head]=code16;
			rxq_head=(rxq_head + 1) & (RXQ_SIZE-1);
			
			// Сбрасываем флаги
			was_E0=was_E1=was_F0=0;
		    }
		}
	    }
	    
	    // Сбрасываем приемник
	    rx=0;
	    rxbit=1;
	}
    }
}


void ps2_init(void) {
    // Переключаем PS2_DATA и PS2_CLK в GPIO
    gpio_init_input_pu(PS2_DATA);
    gpio_init_input_pu(PS2_CLK);
    
    // Настраиваем прерывание по низкому фронту на PS2_CLK
    gpio_pin_intr_state_set(PS2_CLK, GPIO_PIN_INTR_NEGEDGE);
    
    // Настраиваем прерывание по GPIO
    ETS_GPIO_INTR_ATTACH(gpio_int, 0);
    ETS_GPIO_INTR_ENABLE();
    
    // Задача периодики
    PT_INIT(&pt_task);
}

uint32_t ps2get_raw_code(); // TODO: remap?
#include "nespad.h"
#include "ps2_rk.h"

static uint16_t int_ps2_read(void) {
	static uint16_t tcc = 0;
	uint16_t w;
	if (nespad_state2 && !nespad_state) nespad_state = nespad_state2;
	if (nespad_state) {
		if (nespad_state & DPAD_RIGHT) {
			w = PS2_RIGHT;
		} else if (nespad_state & DPAD_LEFT) {
			w = PS2_LEFT;
		} else if (nespad_state & DPAD_UP) {
			w = PS2_UP;
		} else if (nespad_state & DPAD_DOWN) {
			w = PS2_DOWN;
		} else if (nespad_state & DPAD_A) {
			w = PS2_ENTER;
		} else if (nespad_state & DPAD_B) {
			w = PS2_SPACE;
		} else if (nespad_state & DPAD_SELECT) {
			w = PS2_ESC;
		} else if (nespad_state & DPAD_START) {
			w = PS2_F12;
		}
		if (tcc) {
			if (w != tcc) { // конпка сменилась, надо послать "отжатие"
				w = tcc | 0x8000;
				tcc = 0;
				return w;			
			}
			return 0; // нажата та же кнопка, ничего не возвращаем
		}
		tcc = w; // запоминаем, что нажато было
		return w;
	}
	if (tcc) {
		w = tcc | 0x8000;
		tcc = 0;
		return w;
	}
	w = (uint16_t)ps2get_raw_code() & 0xFFFF;
	if (w & 0xF000) w = (w & ~0x7000);
	//if (w) printf("ps2_read: %04Xh", w);
	return w;
}

uint16_t ps2_read(void) {
	static bool ctrl = false;
	static bool alt = false;
	static bool del = false;
	uint16_t c = int_ps2_read();
	switch (c)
	{
	case PS2_L_CTRL:
	case PS2_R_CTRL:
		ctrl = true;
		break;
	case PS2_L_CTRL | 0x8000:
	case PS2_R_CTRL | 0x8000:
		ctrl = false;
		break;
	case PS2_L_ALT:
	case PS2_R_ALT:
		alt = true;
		break;
	case PS2_L_ALT | 0x8000:
	case PS2_R_ALT | 0x8000:
		alt = false;
		break;
	case PS2_DELETE:
	case PS2_KP_PERIOD:
		del = true;
		break;
	case PS2_DELETE | 0x8000:
	case PS2_KP_PERIOD | 0x8000:
		del = false;
		break;
	}
	if (ctrl && alt && del) {
        f_unlink("/.firmware");
        watchdog_enable(1, true);
        while (true);
	}
	return c;
}

void ps2_leds(bool caps, bool num, bool scroll) {
    led_status=(caps ? 0x04 : 0x00) | (num ? 0x02 : 0x00) | (scroll ? 0x01 : 0x00);
}

static void start_tx(uint8_t b) {
    uint8_t p=b;
    p^=p >> 4;
    p^=p >> 2;
    p^=p >> 1;
    tx=b | ((p & 0x01) ? 0x000 : 0x100);
    txbit=1;
}


static PT_THREAD(task(struct pt *pt))
{
    static uint32_t _sleep;
    static uint8_t last_led=0x00;
    static uint8_t l;
    
#warning TODO: надо сделать буфер для передачи в клавиатуру и обрабатывать resend итд
    PT_BEGIN(pt);
	while (1)
	{
	    if ( (last_led == led_status) && (! bat) )
	    {
		// Лампочки не изменились
		PT_YIELD(pt);
		continue;
	    }
	    bat=0;
	    
	    //ets_printf("PS2: sending leds 0x%02X\n", led_status);
	    
resend1:
	    // PS2_CLK вниз
	    gpio_off(PS2_CLK);
	    gpio_init_output(PS2_CLK);
	    PT_SLEEP(100);
	    
	    // PS2_DATA вниз (старт бит)
	    gpio_off(PS2_DATA);
	    gpio_init_output(PS2_DATA);
	    PT_SLEEP(200);
	    
	    // Отправляем команду "Set/Reset LEDs"
	    ack=0;
	    resend=0;
	    start_tx(0xED);
	    
	    // Отпускаем PS2_CLK
	    gpio_init_input_pu(PS2_CLK);
	    
	    // Ждем немного
	    PT_SLEEP(5000);
	    
	    // Проверим подтверждение
	    if (resend) goto resend1;
	    if (! ack) continue;
	    
	    
resend2:
	    // PS2_CLK вниз
	    gpio_off(PS2_CLK);
	    gpio_init_output(PS2_CLK);
	    PT_SLEEP(100);
	    
	    // PS2_DATA вниз (старт бит)
	    gpio_off(PS2_DATA);
	    gpio_init_output(PS2_DATA);
	    PT_SLEEP(200);
	    
	    // Отправляем лампочки
	    ack=0;
	    resend=0;
	    l=led_status;
	    start_tx(l);
	    
	    // Отпускаем PS2_CLK
	    gpio_init_input_pu(PS2_CLK);
	    
	    // Ждем немного
	    PT_SLEEP(5000);
	    
	    // Проверим подтверждение
	    if (resend) goto resend2;
	    if (! ack) continue;
	    
	    // Сохраняем отправленное состояние
	    last_led=l;
	}
    PT_END(pt);
}
