/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rgbled_aw2013.cpp
 *
 * Driver for the onboard RGB LED controller (AW2013)
 * connected via I2C.
 *
 * @author AMOVeason <eason473867143@gmail.com>
 */

#include <string.h>

#include <drivers/device/i2c.h>
#include <lib/led/led.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

#define swap16(w)                       __builtin_bswap16((w))

/**
 * @brief register address
 *
 */
#define AW2013_REG_RSTR                      0x00      /**< soft reset register*/
#define AW2013_REG_GCR                       0x01      /**< global control register*/
#define AW2013_REG_LCTR                      0x30      /**< led control register*/
#define AW2013_REG_CONFIG_LED0               0x31      /**< blue led mode control register*/
#define AW2013_REG_CONFIG_LED1               0x32      /**< green led mode control register*/
#define AW2013_REG_CONFIG_LED2               0x33      /**< red led mode control register*/

#define AW2013_REG_LED_PWM_BLUE		     0x34      /** BLUE */
#define AW2013_REG_LED_PWM_GREEN	     0x35      /** GREEN */
#define AW2013_REG_LED_PWM_RED		     0x36     /** RED */

#define AW2013_ADDR                          0x45      /**< I2C address of AW2013   */


#define AW2013_LED_T1_T2_BLUE                0x37       /*淡入*/
#define AW2013_LED_T1_T2_GREEN               0x3a
#define AW2013_LED_T1_T2_RED                 0x3d

#define AW2013_LED_T3_T4_BLUE                0x38       /*淡出 */
#define AW2013_LED_T3_T4_GREEN               0x3b
#define AW2013_LED_T3_T4_RED                 0x3e


#define AW2013_LED_T0_BLUE                   0x39       /*重复的次数*/
#define AW2013_LED_T0_GREEN                  0x3c
#define AW2013_LED_T0_RED                    0x3f

/**
 * @brief register bits
 *
 */
#define AW2013_LED_RESET    		     0x55
#define AW2013_CHIPID       		     0x33  /**< soft reset I2C recorized id*/

#define AW2013_LED_ENABLE_RED    	     0x04
#define AW2013_LED_ENABLE_GREEN  	     0x02
#define AW2013_LED_ENABLE_BLUE   	     0x01  /**<LED function enable */

#define AW2013_LED_CONTROL_ENABLE    	     0x07  /**< LED channel enable */
#define AW2013_LED_CONFIG_MODE 		     0x03  /**<LED mode control config */
#define AW2013_LED_BRIGHT  		     0xa0  /**< full brightness */

#define AW_LED_RESET_DELAY		8

class RGBLED_AW2013 : public device::I2C, public I2CSPIDriver<RGBLED_AW2013>
{
public:
	RGBLED_AW2013(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address);
	virtual ~RGBLED_AW2013() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int		init() override;
	int		probe() override;

	void			RunImpl();

//protected:
//	void			print_status() override;
private:
	int			send_led_rgb();
	void			update_params();

	//int			write(uint8_t reg, uint8_t data);

	uint8_t			_brightness{0};
	float_t			_max_brightness{1.0f};

	uint8_t		_r{0};
	uint8_t		_g{0};
	uint8_t		_b{0};
	volatile bool		_running{false};
	volatile bool		_should_run{true};
	bool			_led_opened{true};



	int		send_led_enable(uint8_t color_ ); // R G B
	int		send_led_open(bool open);
	int		send_led_brightness_mode();
	int     	set_breathe_mode(bool type,uint8_t rgb_control);
	int     	set_fade_out(uint8_t time,uint8_t off_time,uint8_t rgb_control);
	int     	set_fade_in(uint8_t time,uint8_t on_time,uint8_t rgb_control);
	int     	set_repeat_time(uint8_t start_time,uint8_t count,uint8_t rgb_control);

	int 		read(uint8_t address, int16_t &data);
	int 		write(uint8_t address, uint16_t data);
	int      	get_chip_id();

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	LedController		_led_controller;

};

RGBLED_AW2013::RGBLED_AW2013(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address) :
	I2C(DRV_LED_DEVTYPE_RGBLED_AW2013, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
{
}

int
RGBLED_AW2013::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}


	/* switch off LED on start */

	send_led_enable(0);
	send_led_open(true);

	send_led_rgb();

	update_params();

	ScheduleNow();

	return OK;
}

int
RGBLED_AW2013::probe()
{
	int ret;
	// bool on, powersave;
	// uint8_t r, g, b;

	/**
	   this may look strange, but is needed. There is a serial
	   EEPROM (Microchip-24aa01) that responds to a bunch of I2C
	   addresses, including the 0x55 used by this LED device. So
	   we need to do enough operations to be sure we are talking
	   to the right device. These 3 operations seem to be enough,
	   as the 3rd one consistently fails if no RGBLED is on the bus.
	 */

	unsigned prevretries = _retries;
	_retries = 4;

	if ((ret = send_led_enable(0) != OK) ||
	    (ret = send_led_enable(0) != OK)) {
		return ret;
	}

	_retries = prevretries;

	return ret;
}

void
RGBLED_AW2013::RunImpl()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();

		// Immediately update to change brightness
		send_led_rgb();
	}

	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			send_led_brightness_mode();
			send_led_rgb();
			_r = AW2013_LED_BRIGHT; _g = 0; _b = 0;
			send_led_enable(AW2013_LED_ENABLE_RED);
			break;

		case led_control_s::COLOR_GREEN:
			send_led_brightness_mode();
			send_led_rgb();
			_r = 0; _g = AW2013_LED_BRIGHT; _b = 0;
			send_led_enable(AW2013_LED_ENABLE_GREEN);
			break;

		case led_control_s::COLOR_BLUE:
			send_led_brightness_mode();
			send_led_rgb();
			_r = 0; _g = 0; _b = AW2013_LED_BRIGHT;
			send_led_enable(AW2013_LED_ENABLE_BLUE);
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			send_led_brightness_mode();
			send_led_rgb();
			_r = AW2013_LED_BRIGHT; _g = AW2013_LED_BRIGHT; _b = 0;
			send_led_enable(AW2013_LED_ENABLE_RED|AW2013_LED_ENABLE_GREEN);
			break;

		case led_control_s::COLOR_PURPLE:
			send_led_brightness_mode();
			send_led_rgb();
			_r = AW2013_LED_BRIGHT; _g = 0; _b = AW2013_LED_BRIGHT;
			send_led_enable(AW2013_LED_ENABLE_RED|AW2013_LED_ENABLE_BLUE);
			break;

		case led_control_s::COLOR_CYAN:
			send_led_brightness_mode();
			send_led_rgb();
			_r = 0; _g = AW2013_LED_BRIGHT; _b = AW2013_LED_BRIGHT;
			send_led_enable(AW2013_LED_ENABLE_GREEN|AW2013_LED_ENABLE_BLUE);
			break;

		case led_control_s::COLOR_WHITE:
			send_led_brightness_mode();
			send_led_rgb();
			_r = AW2013_LED_BRIGHT; _g = AW2013_LED_BRIGHT; _b = AW2013_LED_BRIGHT;
			send_led_enable(AW2013_LED_ENABLE_RED|AW2013_LED_ENABLE_GREEN|AW2013_LED_ENABLE_BLUE);
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			send_led_brightness_mode();
			send_led_rgb();
			send_led_enable(0);
			break;
		}
	}
	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/*
    功能: 设置淡入淡出呼吸模式
    参数:
    @type 1是淡出，0是淡入
    @rgb_control rgb寄存器
    返回值: 成功返回0，失败返回-1
*/
int
RGBLED_AW2013::set_breathe_mode(bool type,uint8_t rgb_control)
{
	uint8_t msg[2]={0,0};
	msg[0]=rgb_control;
	if(!_led_opened) return -1;
	if(type)
	{
		msg[1]=1<<6|0x03;
		transfer(msg, 2, nullptr, 0);
	}else
	{
		msg[1]=1<<5|0x03;
		transfer(msg, 2, nullptr, 0);
	}
	return 0;
}
/*
    功能: 设置淡出模式
    参数:
    @time : 从亮到暗的时间  0-16S
    @off_time : 自主呼吸熄灭保持时间 ,8 级可选  0-16S
    @rgb_control : rgb寄存器
    返回值: 成功返回0，失败返回-1
 */
int
RGBLED_AW2013::set_fade_out(uint8_t time,uint8_t off_time,uint8_t rgb_control)
{
	uint8_t msg[2]={0,0};
	msg[0]=rgb_control;
	if(!_led_opened) return -1;
	if(time>0x08||off_time>0x08)  return -1;
	msg[1]=time<<4|off_time;
	transfer(msg, 2, nullptr, 0);
	return 0;
}
/*
    功能:设置淡入模式
    参数:
    @time : 从暗到亮的时间  0-16S
    @on_time 自主呼吸最大亮度保持时间, 6 级可选 0-4S
    @rgb_control :rgb寄存器
    返回值: 成功返回0，失败返回-1
*/
int
RGBLED_AW2013::set_fade_in(uint8_t time,uint8_t on_time,uint8_t rgb_control)
{

	uint8_t msg[2]={0,0};
	msg[0]=rgb_control;
	if(!_led_opened) return -1;
	if(time>0x08||on_time>0x07)  return -1;
	msg[1]=time<<4|on_time;
	transfer(msg, 2, nullptr, 0);
	return 0;
}
/*
    功能:  重复次数
    参数:
    @start_time 设置 LED 自主呼吸延迟启动时间 0-16S
    @count 设置呼吸重复次数    0-15 次
    @rgb_control  rgb寄存器
    返回值:成功返回0，失败返回-1
*/
int
RGBLED_AW2013::set_repeat_time(uint8_t start_time,uint8_t count,uint8_t rgb_control)
{
	uint8_t msg[2]={0,0};
	msg[0]=rgb_control;

	if(!_led_opened) return -1;
	if(start_time>0xf||count>0xf)  return -1;
	msg[1]=start_time<<4|count;
	transfer(msg, 2, nullptr, 0);
	return 0;
}
/*
    功能: 获取芯片ID
    参数:
    @void
    返回值:成功返回0，失败返回-1
*/
int
RGBLED_AW2013::get_chip_id()
{
	uint8_t buf[2]={0,0};
	uint8_t addr=0x00;
	transfer(&addr,1 ,buf,1);
	if(buf[0]!=0x33)
	{
		PX4_DEBUG("aw2013 chipID read error !\n");
		return -1;
	}
	else{
		PX4_INFO_RAW("aw2013 init ok! \n");
	}
	return 0;
}

/**
 * Sent ENABLE flag to LED driveer
 */
int
RGBLED_AW2013::send_led_enable(uint8_t color_)
{
	if(!_led_opened) return -1;

	uint8_t msg[2] = { AW2013_REG_LCTR, color_};
	transfer(msg, sizeof(msg), nullptr, 0);
	return 0;
}

/**
 * @brief LED LCTR reg config enable
 *
 */
int
RGBLED_AW2013::send_led_open(bool open)
{
	uint8_t value;

	_led_opened = open;

	if(open) value=1; else value=0;

	uint8_t msg[2] = { AW2013_REG_GCR, value};

	return transfer(msg, 2, nullptr, 0);

}

/**
 * @brief LED brightness mode config
 *
 */
int
RGBLED_AW2013::send_led_brightness_mode()
{
	uint8_t msg[2]={0,0};

	msg[1]=0x03;

	msg[0]=AW2013_REG_CONFIG_LED0;
	transfer(msg, 2, nullptr, 0);

	msg[0]=AW2013_REG_CONFIG_LED1;
	transfer(msg, 2, nullptr, 0);

	msg[0]=AW2013_REG_CONFIG_LED2;
	transfer(msg, 2, nullptr, 0);
	return 0;
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_AW2013::send_led_rgb()
{
	uint8_t msg[2]={0,0};

	msg[1]=AW2013_LED_BRIGHT;
	msg[0]=AW2013_REG_LED_PWM_RED;
	transfer(msg, 2, nullptr, 0);

	msg[0]=AW2013_REG_LED_PWM_GREEN;
	transfer(msg, 2, nullptr, 0);

	msg[0]=AW2013_REG_LED_PWM_BLUE;
	transfer(msg, 2, nullptr, 0);

	return 0;
}

// not use
void
RGBLED_AW2013::update_params()
{
	int32_t maxbrt = 255;
	param_get(param_find("LED_RGB2_MAXBRT"), &maxbrt);
	maxbrt = maxbrt > 255 ? 255 : maxbrt;
	maxbrt = maxbrt <  0 ?  0 : maxbrt;

	// A minimum of 2 "on" steps is required for breathe effect
	if (maxbrt == 1) {
		maxbrt = 2;
	}

	_max_brightness = maxbrt / 15.0f;
}
void
RGBLED_AW2013::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled_aw2013", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x45);
	// PRINT_MODULE_USAGE_PARAM_INT('o', 123, 123, 321, "RGB PWM Assignment", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *RGBLED_AW2013::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	RGBLED_AW2013 *instance = new RGBLED_AW2013(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

extern "C" __EXPORT int rgbled_aw2013_main(int argc, char *argv[])
{
	using ThisDriver = RGBLED_AW2013;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = AW2013_ADDR;
	// cli.custom1 = 123;
	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli,DRV_LED_DEVTYPE_RGBLED_AW2013);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

