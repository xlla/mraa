/*
 * since Intel discontinues Edison, we have to use another great project 
 * to stay up to date, so we can use latest linux kernel, use latest packages.
 * to continue use mraa & upm library, I have to create this file to work with
 * chardev device.
 * 
 * So I name it revive!
 * 
 * due to the lack of acpi <-> kernel bridge, we can't switch pinmode now.
 * spi/i2c/uart1/pwmX was pre difined in acpi-tables, so I simple remove
 * relative preparation code.
 * 
 * last, I only have Edison Arduino BreadBoard, so I can only write/test code for it.
 * if someone can donated a Edison miniboard, I would like to amend for it.
 * 
 * Author: Xlla <xllacyx@hotmail.com>
 * Copyright (c) 2020
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <errno.h>
#include <time.h>
#include <sys/utsname.h>

#include "common.h"
#include "gpio/gpio_chardev.h"
#include "x86/intel_edison_fab_c_revive.h"

#define PLATFORM_NAME "Intel Edison"
#define SYSFS_CLASS_GPIO "/sys/class/gpio"
#define DEBUGFS_PINMODE_PATH "/sys/kernel/debug/gpio"
#define MAX_SIZE 64
#define MAX_MODE_SIZE 8
#define TRI_STATE_ALL "TRI_STATE_ALL"

// This is an absolute path to a resource file found within sysfs.
// Might not always be correct. First thing to check if mmap stops
// working. Check the device for 0x1199 and Intel Vendor (0x8086)
#define MMAP_PATH "/sys/devices/pci0000:00/0000:00:0c.0/resource0"
#define UART_DEV_PATH "/dev/ttyS1"

typedef struct {
    int sysfs;
    int mode;
} mraa_intel_edison_pindef_t;

typedef struct {
    mraa_intel_edison_pindef_t gpio;
    mraa_intel_edison_pindef_t pwm;
    mraa_intel_edison_pindef_t i2c;
    mraa_intel_edison_pindef_t spi;
    mraa_intel_edison_pindef_t uart;
} mraa_intel_edison_pinmodes_t;

static mraa_gpio_context tristate;

static mraa_intel_edison_pinmodes_t pinmodes[MRAA_INTEL_EDISON_PINCOUNT];
//invent another group magic numbers to location chip and line
static unsigned int outputen[] = { 248, 249, 250, 251, 252, 253, 254, 255, 256, 257,
                                   258, 259, 260, 261, 232, 233, 234, 235, 236, 237 };

static char* directionNames[] = {
    "MUX33_DIR", 
    "MUX31_DIR", 
    "MUX29_DIR", 
    "MUX27_DIR", 
    "MUX24_DIR", 
    "MUX21_DIR", 
    "MUX19_DIR", 
    "MUX32_DIR", 
    "MUX30_DIR", 
    "MUX28_DIR", 
    "MUX26_DIR", 
    "MUX23_DIR", 
    "MUX20_DIR", 
    "MUX18_DIR", 
    "MUX14_DIR", 
    "MUX12_DIR", 
    "MUX10_DIR", 
    "MUX8_DIR", 
    "MUX6_DIR", 
    "MUX4_DIR", 
};


static mraa_gpio_context agpioOutputen[sizeof(outputen) / sizeof(outputen[0])];

static unsigned int pullup_map[] = { 216, 217, 218, 219, 220, 221, 222, 223, 224, 225,
                                     226, 227, 228, 229, 208, 209, 210, 211, 212, 213 };

static char* biasNames[] = {
    "DIG0_PU_PD", 
    "DIG1_PU_PD", 
    "DIG2_PU_PD", 
    "DIG3_PU_PD", 
    "DIG4_PU_PD", 
    "DIG5_PU_PD", 
    "DIG6_PU_PD", 
    "DIG7_PU_PD", 
    "DIG8_PU_PD", 
    "DIG9_PU_PD", 
    "DIG10_PU_PD", 
    "DIG11_PU_PD", 
    "DIG12_PU_PD", 
    "DIG13_PU_PD", 
    "A0_PU_PD", 
    "A1_PU_PD", 
    "A2_PU_PD", 
    "A3_PU_PD", 
    "A4_PU_PD", 
    "A5_PU_PD", 
};
static int miniboard = 0;

// MMAP
static uint8_t* mmap_reg = NULL;
static int mmap_fd = 0;
static int mmap_size;
static unsigned int mmap_count = 0;

// Pin state for PWM 0% duty and enable/disable bug workaround
typedef struct {
    float duty_cycle;
    int pwm_disabled;
} mraa_edison_pwm_wa_pinstate_t;

static mraa_edison_pwm_wa_pinstate_t pwm_wa_state[4] = {{.duty_cycle = 0, .pwm_disabled = 0}};



mraa_boolean_t is_revive()
{
    mraa_boolean_t result = 0;
    struct utsname name;
    int major, minor, release;
    int ret;

    if (uname(&name) != 0) {
        return result;
    }

    ret = sscanf(name.release, "%d.%d.%d", &major, &minor, &release);
    if (ret == 2) {
        ret++;
        release = 0;
    }
    if (ret < 2) {
        return result;
    }

    if ((major == 4 && minor >= 18) ||(major >= 5)) {
        syslog(LOG_NOTICE,
            "edison: Linux version 4.18 or higher detected, use chardev driver");
        result = 1;
    }
    return result;
}

mraa_result_t
mraa_intel_edison_revive_spi_lsbmode_replace(mraa_spi_context dev, mraa_boolean_t lsb)
{
    uint8_t lsb_mode = (uint8_t) lsb;

    // Edison doesn't support LSB_FIRST, we need to react appropriately
    if (!lsb) {
        if (ioctl(dev->devfd, SPI_IOC_WR_LSB_FIRST, &lsb_mode) < 0) {
            syslog(LOG_ERR, "spi: Failed to set bit order");
            return MRAA_ERROR_INVALID_RESOURCE;
        }
        if (ioctl(dev->devfd, SPI_IOC_RD_LSB_FIRST, &lsb_mode) < 0) {
            syslog(LOG_ERR, "spi: Failed to set bit order");
            return MRAA_ERROR_INVALID_RESOURCE;
        }
    } else {
        return MRAA_ERROR_FEATURE_NOT_SUPPORTED;
    }

    dev->lsb = lsb;
    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_intel_edison_revive_pinmode_change(int sysfs, int mode)
{
    if (mode < 0) {
        return MRAA_SUCCESS;
    }

    // return MRAA_SUCCESS;
    char buffer[MAX_SIZE];
    int useDebugFS = 0;

    mraa_gpio_context mode_gpio = mraa_gpio_init_raw(sysfs);
    if (mode_gpio == NULL) {
        return MRAA_ERROR_NO_RESOURCES;
    }

    // first try SYSFS_CLASS_GPIO path
    snprintf(buffer, MAX_SIZE, SYSFS_CLASS_GPIO "/gpio%i/pinmux", sysfs);
    int modef = open(buffer, O_WRONLY);
    if (modef == -1) {
        snprintf(buffer, MAX_SIZE, DEBUGFS_PINMODE_PATH "%i/current_pinmux", sysfs);
        modef = open(buffer, O_WRONLY);
        useDebugFS = 1;
    }

    if (modef == -1) {
        syslog(LOG_ERR, "edison: Failed to open SoC pinmode for opening");
        mraa_gpio_close(mode_gpio);
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    mraa_result_t ret = MRAA_SUCCESS;
    char mode_buf[MAX_MODE_SIZE];
    int length = snprintf(mode_buf, MAX_MODE_SIZE, "%s%u", useDebugFS ? "mode" : "", mode);
    if (write(modef, mode_buf, length * sizeof(char)) == -1) {
        ret = MRAA_ERROR_INVALID_RESOURCE;
    }
    close(modef);
    mraa_gpio_close(mode_gpio);

    return ret;
}

mraa_result_t
mraa_intel_edison_revive_gpio_dir_pre(mraa_gpio_context dev, mraa_gpio_dir_t dir)
{

    if (dev->phy_pin >= 0) {
        if (mraa_gpio_write(tristate, 0) != MRAA_SUCCESS) {
            // call can sometimes fail, this does not actually mean much except
            // that the kernel drivers don't always behave very well
            syslog(LOG_NOTICE, "edison: Failed to write to tristate");
        }
        int pin = dev->phy_pin;

        if (!agpioOutputen[pin]) {
            //will init one chardev gpio pin dev 
            agpioOutputen[pin] = mraa_gpio_init_by_name(directionNames[pin]);
            if (agpioOutputen[pin] == NULL) {
                return MRAA_ERROR_INVALID_RESOURCE;
            }
            if (mraa_gpio_dir(agpioOutputen[pin], MRAA_GPIO_OUT) != MRAA_SUCCESS) {
                return MRAA_ERROR_INVALID_RESOURCE;
            }
        }
        int output_val = 0;
        if (dir == MRAA_GPIO_OUT) {
            output_val = 1;
        }
        if (mraa_gpio_write(agpioOutputen[pin], output_val) != MRAA_SUCCESS) {
            return MRAA_ERROR_INVALID_RESOURCE;
        }
    }

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_gpio_dir_post(mraa_gpio_context dev, mraa_gpio_dir_t dir)
{
    if (dev->phy_pin >= 0) {
        return mraa_gpio_write(tristate, 0);
    }
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_gpio_init_post(mraa_gpio_context dev)
{
    if (dev == NULL) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    int sysfs, mode;
    if (miniboard == 1) {
        sysfs = dev->pin;
        mode = 0;
    } else {
        sysfs = pinmodes[dev->phy_pin].gpio.sysfs;
        mode = pinmodes[dev->phy_pin].gpio.mode;
    }

    return mraa_intel_edison_revive_pinmode_change(sysfs, mode);
}

mraa_result_t
mraa_intel_edison_revive_gpio_close_pre(mraa_gpio_context dev)
{
    // check if we own it
    if (dev->owner != 0) {
        if (dev->phy_pin >= 0) {
            int pin = dev->phy_pin;
            if (agpioOutputen[pin]) {
                mraa_gpio_close(agpioOutputen[pin]);
                agpioOutputen[pin] = NULL;
            }
        }
    }
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_i2c_init_pre(unsigned int bus)
{
    if (miniboard == 0) {
        if (bus != 6) {
            syslog(LOG_ERR, "edison: You can't use that bus, switching to bus 6");
            bus = 6;
        }
        //everything is done in acpi
    } else {
        if (bus != 6 && bus != 1) {
            syslog(LOG_ERR, "edison: You can't use that bus, switching to bus 6");
            bus = 6;
        }
        syslog(LOG_ERR, "edison: current only Edison Arduino breadboard supported!");
        return MRAA_ERROR_INVALID_RESOURCE;
        // int scl = plat->pins[plat->i2c_bus[bus].scl].gpio.pinmap;
        // int sda = plat->pins[plat->i2c_bus[bus].sda].gpio.pinmap;
        // mraa_intel_edison_pinmode_change(sda, 1);
        // mraa_intel_edison_pinmode_change(scl, 1);
    }

    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_intel_edison_revive_misc_spi()
{
    //everything is done in acpi
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_aio_get_fp(mraa_aio_context dev)
{
    char file_path[64] = "";

    snprintf(file_path, 64, "/sys/bus/iio/devices/iio:device1/in_voltage%d_raw", dev->channel);

    dev->adc_in_fp = open(file_path, O_RDONLY);
    if (dev->adc_in_fp == -1) {
        syslog(LOG_ERR, "edison: Failed to open Analog input raw file %s for "
                        "reading!",
               file_path);
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_aio_init_pre(unsigned int aio)
{
    if (aio > plat->aio_count) {
        syslog(LOG_ERR, "edison: Invalid analog input channel");
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    int pin = 14 + aio;
    mraa_gpio_context output_e;
    output_e = mraa_gpio_init_by_name(directionNames[pin]);
    if (output_e == NULL) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    if (mraa_gpio_dir(output_e, MRAA_GPIO_OUT) != MRAA_SUCCESS) {
        mraa_gpio_close(output_e);
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    if (mraa_gpio_write(output_e, 0) != MRAA_SUCCESS) {
        mraa_gpio_close(output_e);
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    mraa_gpio_close(output_e);

    mraa_gpio_context pullup_pin;
    pullup_pin = mraa_gpio_init_by_name(biasNames[pin]);
    if (pullup_pin == NULL) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    if (mraa_gpio_dir(pullup_pin, MRAA_GPIO_IN) != MRAA_SUCCESS) {
        mraa_gpio_close(pullup_pin);
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    mraa_gpio_close(pullup_pin);

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_aio_init_post(mraa_aio_context dev)
{
    return mraa_gpio_write(tristate, 1);
}

mraa_result_t
mraa_intel_edison_revive_pwm_enable_pre(mraa_pwm_context dev, int enable) {
    // PWM 0% duty workaround: update state array
    // if someone first ran write(0) and then enable(1).
    if ((pwm_wa_state[dev->pin].pwm_disabled == 1) && (enable == 1)) {
        pwm_wa_state[dev->pin].pwm_disabled = 0;
        return MRAA_SUCCESS;
    }

    if (enable == 0) {
        // Set duty cycle to 0 before disabling PWM, but save it first
        pwm_wa_state[dev->pin].duty_cycle = mraa_pwm_read(dev);
        // Edison PWM output stuck at high if disabled during ON period
        mraa_pwm_pulsewidth_us(dev, 0);
        // Sleep for 2 periods to allow the change to take effect
        usleep(dev->period / 500);
    } else if (enable == 1) {
        // Restore the duty before re-enabling, but not if it's 0, to avoid recursion
        if (pwm_wa_state[dev->pin].duty_cycle != 0) {
            mraa_pwm_write(dev, pwm_wa_state[dev->pin].duty_cycle);
        }
    }

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_pwm_write_pre(mraa_pwm_context dev, float percentage) {
    // PWM 0% duty workaround: set the state array and enable/disable pin accordingly
    if (percentage == 0.0f) {
        syslog(LOG_INFO, "edison_pwm_write_pre (pwm%i): requested zero duty cycle, disabling PWM on the pin", dev->pin);
        pwm_wa_state[dev->pin].pwm_disabled = 1;
        return mraa_pwm_enable(dev, 0);
    } else if (pwm_wa_state[dev->pin].pwm_disabled == 1) {
        syslog(LOG_INFO, "edison_pwm_write_pre (pwm%i): Re-enabling the pin after setting non-zero duty", dev->pin);
        pwm_wa_state[dev->pin].pwm_disabled = 0;
        return mraa_pwm_enable(dev, 1);
    }

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_pwm_init_pre(int pin)
{
    if (miniboard == 1) {
        return mraa_intel_edison_revive_pinmode_change(plat->pins[pin].gpio.pinmap, 1);
    }
    if (pin < 0 || pin > 19) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    if (!plat->pins[pin].capabilities.pwm) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    mraa_gpio_context output_e;
    output_e = mraa_gpio_init_by_name(directionNames[pin]);
    if (output_e == NULL) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    if (mraa_gpio_dir(output_e, MRAA_GPIO_OUT) != MRAA_SUCCESS) {
        mraa_gpio_close(output_e);
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    if (mraa_gpio_write(output_e, 1) != MRAA_SUCCESS) {
        mraa_gpio_close(output_e);
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    mraa_gpio_close(output_e);

    mraa_gpio_context pullup_pin;
    pullup_pin = mraa_gpio_init_by_name(biasNames[pin]);
    if (pullup_pin == NULL) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    if (mraa_gpio_dir(pullup_pin, MRAA_GPIO_IN) != MRAA_SUCCESS) {
        mraa_gpio_close(pullup_pin);
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    mraa_gpio_close(pullup_pin);
    mraa_intel_edison_revive_pinmode_change(plat->pins[pin].gpio.pinmap, 1);

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_pwm_init_post(mraa_pwm_context pwm)
{
    pwm_wa_state[pwm->pin].pwm_disabled = 0;
    pwm_wa_state[pwm->pin].duty_cycle = 0.0f;
    return mraa_gpio_write(tristate, 1);
}

mraa_result_t
mraa_intel_edison_revive_spi_init_pre(int bus)
{
    if (miniboard == 1) {
        syslog(LOG_ERR, "edison: current only Edison Arduino breadboard supported!");
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    //everything is done in acpi
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_spi_init_post(mraa_spi_context spi)
{
    //everything is done in acpi
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_gpio_mode_replace(mraa_gpio_context dev, mraa_gpio_mode_t mode)
{
    if (dev->value_fp != -1) {
        if (close(dev->value_fp) != 0) {
            return MRAA_ERROR_INVALID_RESOURCE;
        }
        dev->value_fp = -1;
    }

    mraa_gpio_context pullup_e;
    pullup_e = mraa_gpio_init_by_name(biasNames[dev->phy_pin]);
    if (pullup_e == NULL) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    if (mraa_gpio_dir(pullup_e, MRAA_GPIO_IN) != MRAA_SUCCESS) {
        syslog(LOG_ERR, "edison: Failed to set gpio mode-pullup");
        mraa_gpio_close(pullup_e);
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    int value = -1;
    switch (mode) {
        case MRAA_GPIO_STRONG:
            break;
        case MRAA_GPIO_PULLUP:
            value = 1;
            break;
        case MRAA_GPIO_PULLDOWN:
            value = 0;
            break;
        case MRAA_GPIO_HIZ:
            break;
        default:
            return MRAA_ERROR_FEATURE_NOT_IMPLEMENTED;
    }
    if (value != -1) {
        if (mraa_gpio_dir(pullup_e, MRAA_GPIO_OUT) != MRAA_SUCCESS) {
            syslog(LOG_ERR, "edison: Error setting pullup");
            mraa_gpio_close(pullup_e);
            return MRAA_ERROR_INVALID_RESOURCE;
        }
        if (mraa_gpio_write(pullup_e, value) != MRAA_SUCCESS) {
            syslog(LOG_ERR, "edison: Error setting pullup");
            mraa_gpio_close(pullup_e);
            return MRAA_ERROR_INVALID_RESOURCE;
        }
    }

    return mraa_gpio_close(pullup_e);
}

mraa_result_t
mraa_intel_edison_revive_mb_gpio_mode(mraa_gpio_context dev, mraa_gpio_mode_t mode)
{
    if (dev->value_fp != -1) {
        if (close(dev->value_fp) != 0) {
            return MRAA_ERROR_INVALID_RESOURCE;
        }
        dev->value_fp = -1;
    }

    char filepath[MAX_SIZE];

    mraa_gpio_context mode_gpio = mraa_gpio_init_raw(dev->pin);
    if (mode_gpio == NULL) {
        return MRAA_ERROR_NO_RESOURCES;
    }

    // first try SYSFS_CLASS_GPIO path
    snprintf(filepath, MAX_SIZE, SYSFS_CLASS_GPIO "/gpio%d/pullmode", dev->pin);
    int drive = open(filepath, O_WRONLY);

    if (drive == -1) {
        snprintf(filepath, MAX_SIZE, DEBUGFS_PINMODE_PATH "%d/current_pullmode", dev->pin);
        drive = open(filepath, O_WRONLY);
    }

    if (drive == -1) {
        syslog(LOG_ERR, "edison: Failed to open drive for writing");
        mraa_gpio_close(mode_gpio);
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    char bu[MAX_SIZE];
    int length;
    switch (mode) {
        case MRAA_GPIO_STRONG:
            mraa_gpio_close(mode_gpio);
            close(drive);
            return MRAA_SUCCESS;
        case MRAA_GPIO_PULLUP:
            length = snprintf(bu, sizeof(bu), "pullup");
            break;
        case MRAA_GPIO_PULLDOWN:
            length = snprintf(bu, sizeof(bu), "pulldown");
            break;
        case MRAA_GPIO_HIZ:
            length = snprintf(bu, sizeof(bu), "nopull");
            break;
        default:
            mraa_gpio_close(mode_gpio);
            close(drive);
            return MRAA_ERROR_FEATURE_NOT_IMPLEMENTED;
    }
    if (write(drive, bu, length * sizeof(char)) == -1) {
        syslog(LOG_ERR, "edison: Failed to write to drive mode");
        mraa_gpio_close(mode_gpio);
        close(drive);
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    mraa_gpio_close(mode_gpio);
    if (close(drive) != 0) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_uart_init_pre(int index)
{
    if (index != 0) {
        syslog(LOG_ERR, "edison: Failed to write to drive mode");
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    if (miniboard == 0) {
        //everything is done in acpi
    }
    //everything is done in acpi
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_uart_init_post(mraa_uart_context uart)
{
    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_intel_edison_revive_mmap_unsetup()
{
    if (mmap_reg == NULL) {
        syslog(LOG_ERR, "edison mmap: null register cant unsetup");
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    munmap(mmap_reg, mmap_size);
    mmap_reg = NULL;
    if (close(mmap_fd) != 0) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_mmap_write(mraa_gpio_context dev, int value)
{
    uint8_t offset = ((dev->pin / 32) * sizeof(uint32_t));
    uint8_t valoff;

    if (value) {
        valoff = 0x34;
    } else {
        valoff = 0x4c;
    }

    *(volatile uint32_t*) (mmap_reg + offset + valoff) = (uint32_t)(1 << (dev->pin % 32));

    return MRAA_SUCCESS;
}

int
mraa_intel_edison_revive_mmap_read(mraa_gpio_context dev)
{
    uint8_t offset = ((dev->pin / 32) * sizeof(uint32_t));
    uint32_t value;

    value = *(volatile uint32_t*) (mmap_reg + 0x04 + offset);
    if (value & (uint32_t)(1 << (dev->pin % 32))) {
        return 1;
    }
    return 0;
}

mraa_result_t
mraa_intel_edison_revive_mmap_setup(mraa_gpio_context dev, mraa_boolean_t en)
{
    if (dev == NULL) {
        syslog(LOG_ERR, "edison mmap: context not valid");
        return MRAA_ERROR_INVALID_HANDLE;
    }

    if (en == 0) {
        if (dev->mmap_write == NULL && dev->mmap_read == NULL) {
            syslog(LOG_ERR, "edison mmap: can't disable disabled mmap gpio");
            return MRAA_ERROR_INVALID_PARAMETER;
        }
        dev->mmap_write = NULL;
        dev->mmap_read = NULL;
        mmap_count--;
        if (mmap_count == 0) {
            return mraa_intel_edison_revive_mmap_unsetup();
        }
        return MRAA_SUCCESS;
    }

    if (dev->mmap_write != NULL && dev->mmap_read != NULL) {
        syslog(LOG_ERR, "edison mmap: can't enable enabled mmap gpio");
        return MRAA_ERROR_INVALID_PARAMETER;
    }

    // Might need to make some elements of this thread safe.
    // For example only allow one thread to enter the following block
    // to prevent mmap'ing twice.
    if (mmap_reg == NULL) {
        if ((mmap_fd = open(MMAP_PATH, O_RDWR)) < 0) {
            syslog(LOG_ERR, "edison map: unable to open resource0 file");
            return MRAA_ERROR_INVALID_HANDLE;
        }

        struct stat fd_stat;
        if (fstat(mmap_fd, &fd_stat) != 0) {
            syslog(LOG_ERR, "edison map: unable to access resource0 file");
            return MRAA_ERROR_INVALID_HANDLE;
        }
        mmap_size = fd_stat.st_size;

        mmap_reg =
        (uint8_t*) mmap(NULL, fd_stat.st_size, PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, mmap_fd, 0);
        if (mmap_reg == MAP_FAILED) {
            syslog(LOG_ERR, "edison mmap: failed to mmap");
            mmap_reg = NULL;
            close(mmap_fd);
            return MRAA_ERROR_NO_RESOURCES;
        }
    }
    dev->mmap_write = &mraa_intel_edison_revive_mmap_write;
    dev->mmap_read = &mraa_intel_edison_revive_mmap_read;
    mmap_count++;

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_revive_i2c_freq(mraa_i2c_context dev, mraa_i2c_mode_t mode)
{
    syslog(LOG_WARNING, "edison: should set freq by acpi");
    return MRAA_SUCCESS;
}

mraa_gpio_context
mraa_gpio_line_init_by_name(char* name)
{
    mraa_board_t* board = plat;
    mraa_gpio_context dev;
    mraa_gpiod_group_t gpio_group;
    mraa_gpiod_line_info* linfo = NULL;
    mraa_gpiod_chip_info* cinfo;
    mraa_gpiod_chip_info** cinfos;
    int i, line_found = 0, line_offset;

    if (name == NULL) {
        syslog(LOG_ERR, "[GPIOD_INTERFACE]: Gpio name not valid");
        return NULL;
    }

    if (!board->chardev_capable) {
        syslog(LOG_ERR, "[GPIOD_INTERFACE]: gpio_init_by_name not available for this platform!");
        return NULL;
    }

    dev = (mraa_gpio_context) calloc(1, sizeof(struct _gpio));
    if (dev == NULL) {
        syslog(LOG_CRIT, "[GPIOD_INTERFACE]: Failed to allocate memory for context");
        return NULL;
    }

    dev->pin_to_gpio_table = malloc(sizeof(int));
    if (dev->pin_to_gpio_table == NULL) {
        syslog(LOG_CRIT, "[GPIOD_INTERFACE]: Failed to allocate memory for internal member");
        mraa_gpio_close(dev);
        return NULL;
    }

    dev->num_chips = mraa_get_chip_infos(&cinfos);
    if (dev->num_chips <= 0) {
        mraa_gpio_close(dev);
        return NULL;
    }

    /* We are dealing with a single GPIO */
    dev->num_pins = 1;

    gpio_group = calloc(dev->num_chips, sizeof(struct _gpio_group));
    if (gpio_group == NULL) {
        syslog(LOG_CRIT, "[GPIOD_INTERFACE]: Failed to allocate memory for internal member");
        mraa_gpio_close(dev);
        return NULL;
    }

    dev->gpio_group = gpio_group;
    for (i = 0; i < dev->num_chips; ++i) {
        gpio_group[i].gpio_chip = i;
        gpio_group[i].gpio_lines = NULL;
    }

    /* Iterate over all gpiochips in the platform to find the requested line */
    for (int idx = 0; idx < dev->num_chips && (cinfo = cinfos[idx]); (idx++))
    {
        for (i = 0; i < cinfo->chip_info.lines; i++) {
            linfo = mraa_get_line_info_by_chip_name(cinfo->chip_info.name, i);
            if (!strncmp(linfo->name, name, 32)) {
                /* idx is coming from `for_each_gpio_chip` definition */
                syslog(LOG_DEBUG, "[GPIOD_INTERFACE]: Chip: %d Line: %d", idx, i);
                if (!gpio_group[idx].is_required) {
                    gpio_group[idx].dev_fd = cinfo->chip_fd;
                    gpio_group[idx].is_required = 1;
                    gpio_group[idx].gpiod_handle = -1;
                }

                /* Map pin to _gpio_group structure. */
                dev->pin_to_gpio_table[0] = idx;
                gpio_group[idx].gpio_lines = realloc(gpio_group[idx].gpio_lines, sizeof(unsigned int));
                gpio_group[idx].gpio_lines[0] = i;
                gpio_group[idx].num_gpio_lines++;

                line_found = 1;
                line_offset = i;

                break;
            }
        }
        if(line_found) break;
    }

    if (!line_found) {
        syslog(LOG_ERR, "[GPIOD_INTERFACE]: Gpio not found!");
        return NULL;
    }

    /* Initialize rw_values for read / write multiple functions */
    for (i = 0; i < dev->num_chips; ++i) {
        gpio_group[i].rw_values = calloc(gpio_group[i].num_gpio_lines, sizeof(unsigned char));
        if (gpio_group[i].rw_values == NULL) {
            syslog(LOG_CRIT, "[GPIOD_INTERFACE]: Failed to allocate memory for internal member");
            mraa_gpio_close(dev);
            return NULL;
        }

        gpio_group[i].event_handles = NULL;
    }

    /* Save the provided array from the user to our internal structure. */
    dev->provided_pins = malloc(dev->num_pins * sizeof(int));
    if (dev->provided_pins == NULL) {
        syslog(LOG_CRIT, "[GPIOD_INTERFACE]: Failed to allocate memory for internal member");
        mraa_gpio_close(dev);
        return NULL;
    }

    memcpy(dev->provided_pins, &line_offset, dev->num_pins * sizeof(int));

    dev->events = NULL;

    return dev;
}

mraa_result_t is_exist_line(const char* name) {
    int line_gpio = -1;
    if (name == NULL) {
        syslog(LOG_ERR, "[GPIOD_INTERFACE]: Gpio name not valid");
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    mraa_gpio_context dev = mraa_gpio_line_init_by_name(name);
    if (dev) {
        mraa_gpio_close(dev);
    } else {
        syslog(LOG_ERR, "[gpio line check]: %s not found!", name);
        return MRAA_ERROR_INVALID_RESOURCE;
    }

            return MRAA_SUCCESS;
}

mraa_boolean_t
is_arduino_board_revive()
{
    int input_gpio = -1;
    // We check for two things to determine if that's an Arduino expansion board
    // 1) is tristate GPIO available, by trying to initialize it
    // 2) are there four specific GPIO chip, by reading chip name
    const int gpiochip_idx[4] = {  0,  1,  2,  3};
    const char* gpio_line_names[4] = {"MUX15_SEL", "DIG0_PU_PD", "MUX14_DIR", "MUX33_DIR"};

    // check tristate first
    tristate = mraa_gpio_init_by_name(TRI_STATE_ALL);

    if (tristate == NULL) {
        syslog(LOG_INFO, "edison: tristate not detected");
        return 0;
    }
    mraa_result_t r = mraa_gpio_dir(tristate, MRAA_GPIO_OUT);
    if (r != MRAA_SUCCESS) {
    mraa_result_print(r);
    }
    // GPIO expanders second
    for (int i=0; i<(sizeof(gpiochip_idx)/sizeof(gpiochip_idx[0])); i++) {
        // we want to check for exact match
        if (is_exist_line(gpio_line_names[i]) != MRAA_SUCCESS) {
            syslog(LOG_INFO, "edison: %s line not detected", gpio_line_names[i]);
            return 0;
        }
    }

    syslog(LOG_NOTICE, "edison: Arduino board revive detected");
    return 1;
}

mraa_board_t*
mraa_intel_edison_fab_c_revive()
{
    mraa_gpio_dir_t tristate_dir;

    if(!is_revive()) {
        syslog(LOG_CRIT, "edison: Arduino board kernel need >= 4.18");
        return NULL;
    }

    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        return NULL;
    }

    b->platform_name = PLATFORM_NAME;

    plat = b;
    b->chardev_capable = 1;
    // plat is null , can't call chardev functions
    if (is_arduino_board_revive() == 0) {
        syslog(LOG_NOTICE,
               "edison: Arduino board not detected, assuming Intel Edison Miniboard");
        goto error;
    }

    // Now Assuming the edison is attached to the Arduino board.
    b->phy_pin_count = 20;
    b->gpio_count = 14;
    b->aio_count = 6;
    b->platform_version = "arduino";

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        goto error;
    }
    b->adv_func->gpio_dir_pre = &mraa_intel_edison_revive_gpio_dir_pre;
    b->adv_func->gpio_init_post = &mraa_intel_edison_revive_gpio_init_post;
    b->adv_func->gpio_close_pre = &mraa_intel_edison_revive_gpio_close_pre;
    b->adv_func->gpio_dir_post = &mraa_intel_edison_revive_gpio_dir_post;
    b->adv_func->i2c_init_pre = &mraa_intel_edison_revive_i2c_init_pre;
    b->adv_func->i2c_set_frequency_replace = &mraa_intel_edison_revive_i2c_freq;
    b->adv_func->aio_get_valid_fp = &mraa_intel_edison_revive_aio_get_fp;
    b->adv_func->aio_init_pre = &mraa_intel_edison_revive_aio_init_pre;
    b->adv_func->aio_init_post = &mraa_intel_edison_revive_aio_init_post;
    b->adv_func->pwm_init_pre = &mraa_intel_edison_revive_pwm_init_pre;
    b->adv_func->pwm_init_post = &mraa_intel_edison_revive_pwm_init_post;
    b->adv_func->pwm_enable_pre = &mraa_intel_edison_revive_pwm_enable_pre;
    b->adv_func->pwm_write_pre = &mraa_intel_edison_revive_pwm_write_pre;
    b->adv_func->spi_init_pre = &mraa_intel_edison_revive_spi_init_pre;
    b->adv_func->spi_init_post = &mraa_intel_edison_revive_spi_init_post;
    b->adv_func->gpio_mode_replace = &mraa_intel_edison_revive_gpio_mode_replace;
    b->adv_func->uart_init_pre = &mraa_intel_edison_revive_uart_init_pre;
    b->adv_func->uart_init_post = &mraa_intel_edison_revive_uart_init_post;
    // b->adv_func->gpio_mmap_setup = &mraa_intel_edison_revive_mmap_setup;
    b->adv_func->spi_lsbmode_replace = &mraa_intel_edison_revive_spi_lsbmode_replace;

    b->pins = (mraa_pininfo_t*) calloc(MRAA_INTEL_EDISON_PINCOUNT, sizeof(mraa_pininfo_t));
    if (b->pins == NULL) {
        free(b->adv_func);
        goto error;
    }

    // if (mraa_gpio_read_dir(tristate, &tristate_dir) != MRAA_SUCCESS) {
    //     free(b->pins);
    //     free(b->adv_func);
    //     goto error;
    // }

    // if (tristate_dir != MRAA_GPIO_OUT) {
    //     mraa_gpio_dir(tristate, MRAA_GPIO_OUT);
    // }

    // this is required to initialise not just SPI but also the ADC channels
    mraa_intel_edison_revive_misc_spi();

    b->adc_raw = 12;
    b->adc_supported = 10;
    b->pwm_default_period = 5000;
    b->pwm_max_period = 218453;
    b->pwm_min_period = 1;

    strncpy(b->pins[0].name, "IO0", 8);
    b->pins[0].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[0].gpio.pinmap = 130;
    b->pins[0].gpio.parent_id = 0;
    b->pins[0].gpio.mux_total = 1;
    b->pins[0].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[0].gpio.mux[0].pin = 216;
    b->pins[0].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[0].uart.pinmap = 0;
    b->pins[0].uart.parent_id = 0;
    b->pins[0].uart.mux_total = 0;

    strncpy(b->pins[1].name, "IO1", 8);
    b->pins[1].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[1].gpio.pinmap = 131;
    b->pins[1].gpio.parent_id = 0;
    b->pins[1].gpio.mux_total = 1;
    b->pins[1].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[1].gpio.mux[0].pin = 217;
    b->pins[1].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[1].uart.pinmap = 0;
    b->pins[1].uart.parent_id = 0;
    b->pins[1].uart.mux_total = 0;

    strncpy(b->pins[2].name, "IO2", 8);
    b->pins[2].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[2].gpio.pinmap = 128;
    b->pins[2].gpio.parent_id = 0;
    b->pins[2].gpio.mux_total = 1;
    b->pins[2].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[2].gpio.mux[0].pin = 218;
    b->pins[2].gpio.mux[0].value = MRAA_GPIO_IN;

    strncpy(b->pins[3].name, "IO3", 8);
    b->pins[3].capabilities = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[3].gpio.pinmap = 12;
    b->pins[3].gpio.parent_id = 0;
    b->pins[3].gpio.mux_total = 0;
    b->pins[3].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[3].gpio.mux[0].pin = 219;
    b->pins[3].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[3].pwm.pinmap = 0;
    b->pins[3].pwm.parent_id = 0;
    b->pins[3].pwm.mux_total = 0;

    strncpy(b->pins[4].name, "IO4", 8);
    b->pins[4].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[4].gpio.pinmap = 129;
    b->pins[4].gpio.parent_id = 0;
    b->pins[4].gpio.mux_total = 1;
    b->pins[4].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[4].gpio.mux[0].pin = 220;
    b->pins[4].gpio.mux[0].value = MRAA_GPIO_IN;

    strncpy(b->pins[5].name, "IO5", 8);
    b->pins[5].capabilities = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[5].gpio.pinmap = 13;
    b->pins[5].gpio.parent_id = 0;
    b->pins[5].gpio.mux_total = 1;
    b->pins[5].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[5].gpio.mux[0].pin = 221;
    b->pins[5].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[5].pwm.pinmap = 1;
    b->pins[5].pwm.parent_id = 0;
    b->pins[5].pwm.mux_total = 0;

    strncpy(b->pins[6].name, "IO6", 8);
    b->pins[6].capabilities = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[6].gpio.pinmap = 182;
    b->pins[6].gpio.parent_id = 0;
    b->pins[6].gpio.mux_total = 1;
    b->pins[6].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[6].gpio.mux[0].pin = 222;
    b->pins[6].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[6].pwm.pinmap = 2;
    b->pins[6].pwm.parent_id = 0;
    b->pins[6].pwm.mux_total = 0;

    strncpy(b->pins[7].name, "IO7", 8);
    b->pins[7].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[7].gpio.pinmap = 48;
    b->pins[7].gpio.parent_id = 0;
    b->pins[7].gpio.mux_total = 1;
    b->pins[7].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[7].gpio.mux[0].pin = 223;
    b->pins[7].gpio.mux[0].value = MRAA_GPIO_IN;

    strncpy(b->pins[8].name, "IO8", 8);
    b->pins[8].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[8].gpio.pinmap = 49;
    b->pins[8].gpio.parent_id = 0;
    b->pins[8].gpio.mux_total = 1;
    b->pins[8].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[8].gpio.mux[0].pin = 224;
    b->pins[8].gpio.mux[0].value = MRAA_GPIO_IN;

    strncpy(b->pins[9].name, "IO9", 8);
    b->pins[9].capabilities = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    b->pins[9].gpio.pinmap = 183;
    b->pins[9].gpio.parent_id = 0;
    b->pins[9].gpio.mux_total = 1;
    b->pins[9].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[9].gpio.mux[0].pin = 225;
    b->pins[9].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[9].pwm.pinmap = 3;
    b->pins[9].pwm.parent_id = 0;
    b->pins[9].pwm.mux_total = 0;

    strncpy(b->pins[10].name, "IO10", 8);
    b->pins[10].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[10].gpio.pinmap = 41;
    b->pins[10].gpio.parent_id = 0;
    b->pins[10].gpio.mux_total = 3;
    b->pins[10].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[10].gpio.mux[0].pin = 226;
    b->pins[10].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[10].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[10].gpio.mux[1].pin = 263;
    b->pins[10].gpio.mux[1].value = 1;
    b->pins[10].gpio.mux[2].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[10].gpio.mux[2].pin = 240;
    b->pins[10].gpio.mux[2].value = 0;
    b->pins[10].spi.pinmap = 5;
    b->pins[10].spi.mux_total = 3;
    b->pins[10].spi.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[10].spi.mux[0].pin = 226;
    b->pins[10].spi.mux[0].value = MRAA_GPIO_IN;
    b->pins[10].spi.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[10].spi.mux[1].pin = 263;
    b->pins[10].spi.mux[1].value = 1;
    b->pins[10].spi.mux[2].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[10].spi.mux[2].pin = 240;
    b->pins[10].spi.mux[2].value = 1;

    strncpy(b->pins[11].name, "IO11", 8);
    b->pins[11].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[11].gpio.pinmap = 43;
    b->pins[11].gpio.parent_id = 0;
    b->pins[11].gpio.mux_total = 3;
    b->pins[11].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[11].gpio.mux[0].pin = 227;
    b->pins[11].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[11].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[11].gpio.mux[1].pin = 262;
    b->pins[11].gpio.mux[1].value = 1;
    b->pins[11].gpio.mux[2].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[11].gpio.mux[2].pin = 241;
    b->pins[11].gpio.mux[2].value = 0;
    b->pins[11].spi.pinmap = 5;
    b->pins[11].spi.mux_total = 3;
    b->pins[11].spi.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[11].spi.mux[0].pin = 227;
    b->pins[11].spi.mux[0].value = MRAA_GPIO_IN;
    b->pins[10].spi.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[11].spi.mux[1].pin = 262;
    b->pins[11].spi.mux[1].value = 1;
    b->pins[10].spi.mux[2].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[11].spi.mux[2].pin = 241;
    b->pins[11].spi.mux[2].value = 1;

    strncpy(b->pins[12].name, "IO12", 8);
    b->pins[12].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[12].gpio.pinmap = 42;
    b->pins[12].gpio.parent_id = 0;
    b->pins[12].gpio.mux_total = 2;
    b->pins[12].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[12].gpio.mux[0].pin = 228;
    b->pins[12].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[12].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[12].gpio.mux[1].pin = 242;
    b->pins[12].gpio.mux[1].value = 0;
    b->pins[12].spi.pinmap = 5;
    b->pins[12].spi.mux_total = 2;
    b->pins[12].spi.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[12].spi.mux[0].pin = 228;
    b->pins[12].spi.mux[0].value = MRAA_GPIO_IN;
    b->pins[12].spi.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[12].spi.mux[1].pin = 242;
    b->pins[12].spi.mux[1].value = 1;

    strncpy(b->pins[13].name, "IO13", 8);
    b->pins[13].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[13].gpio.pinmap = 40;
    b->pins[13].gpio.parent_id = 0;
    b->pins[13].gpio.mux_total = 2;
    b->pins[13].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[13].gpio.mux[0].pin = 229;
    b->pins[13].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[13].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[13].gpio.mux[1].pin = 243;
    b->pins[13].gpio.mux[1].value = 0;
    b->pins[13].spi.pinmap = 5;
    b->pins[13].spi.mux_total = 2;
    b->pins[13].spi.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[13].spi.mux[0].pin = 229;
    b->pins[13].spi.mux[0].value = MRAA_GPIO_IN;
    b->pins[13].spi.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[13].spi.mux[1].pin = 243;
    b->pins[13].spi.mux[1].value = 1;

    strncpy(b->pins[14].name, "A0", 8);
    b->pins[14].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[14].aio.pinmap = 0;
    b->pins[14].aio.mux_total = 2;
    b->pins[14].aio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[14].aio.mux[0].pin = 208;
    b->pins[14].aio.mux[0].value = MRAA_GPIO_IN;
    b->pins[14].aio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[14].aio.mux[1].pin = 200;
    b->pins[14].aio.mux[1].value = 1;
    b->pins[14].gpio.pinmap = 44;
    b->pins[14].gpio.mux_total = 2;
    b->pins[14].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[14].gpio.mux[0].pin = 208;
    b->pins[14].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[14].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[14].gpio.mux[1].pin = 200;
    b->pins[14].gpio.mux[1].value = 0;

    strncpy(b->pins[15].name, "A1", 8);
    b->pins[15].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[15].aio.pinmap = 1;
    b->pins[15].aio.mux_total = 2;
    b->pins[15].aio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[15].aio.mux[0].pin = 209;
    b->pins[15].aio.mux[0].value = MRAA_GPIO_IN;
    b->pins[15].aio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[15].aio.mux[1].pin = 201;
    b->pins[15].aio.mux[1].value = 1;
    b->pins[15].gpio.pinmap = 45;
    b->pins[15].gpio.mux_total = 2;
    b->pins[15].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[15].gpio.mux[0].pin = 209;
    b->pins[15].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[15].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[15].gpio.mux[1].pin = 201;
    b->pins[15].gpio.mux[1].value = 0;

    strncpy(b->pins[16].name, "A2", 8);
    b->pins[16].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[16].aio.pinmap = 2;
    b->pins[16].aio.mux_total = 2;
    b->pins[16].aio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[16].aio.mux[0].pin = 210;
    b->pins[16].aio.mux[0].value = MRAA_GPIO_IN;
    b->pins[16].aio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[16].aio.mux[1].pin = 202;
    b->pins[16].aio.mux[1].value = 1;
    b->pins[16].gpio.pinmap = 46;
    b->pins[16].gpio.mux_total = 2;
    b->pins[16].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[16].gpio.mux[0].pin = 210;
    b->pins[16].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[16].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[16].gpio.mux[1].pin = 202;
    b->pins[16].gpio.mux[1].value = 0;

    strncpy(b->pins[17].name, "A3", 8);
    b->pins[17].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 1, 0 };
    b->pins[17].aio.pinmap = 3;
    b->pins[17].aio.mux_total = 2;
    b->pins[17].aio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[17].aio.mux[0].pin = 211;
    b->pins[17].aio.mux[0].value = MRAA_GPIO_IN;
    b->pins[17].aio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[17].aio.mux[1].pin = 203;
    b->pins[17].aio.mux[1].value = 1;
    b->pins[17].gpio.pinmap = 47;
    b->pins[17].gpio.mux_total = 2;
    b->pins[17].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[17].gpio.mux[0].pin = 211;
    b->pins[17].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[17].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[17].gpio.mux[1].pin = 203;
    b->pins[17].gpio.mux[1].value = 0;

    strncpy(b->pins[18].name, "A4", 8);
    b->pins[18].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 1, 0 };
    b->pins[18].i2c.pinmap = 1;
    b->pins[18].i2c.mux_total = 2;
    b->pins[18].i2c.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[18].i2c.mux[0].pin = 212;
    b->pins[18].i2c.mux[0].value = MRAA_GPIO_IN;
    b->pins[18].i2c.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[18].i2c.mux[1].pin = 204;
    b->pins[18].i2c.mux[1].value = 0;
    b->pins[18].aio.pinmap = 4;
    b->pins[18].aio.mux_total = 2;
    b->pins[18].aio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[18].aio.mux[0].pin = 212;
    b->pins[18].aio.mux[0].value = MRAA_GPIO_IN;
    b->pins[18].aio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[18].aio.mux[1].pin = 204;
    b->pins[18].aio.mux[1].value = 1;
    b->pins[18].gpio.pinmap = 14;
    b->pins[18].gpio.mux_total = 2;
    b->pins[18].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[18].gpio.mux[0].pin = 212;
    b->pins[18].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[18].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[18].gpio.mux[1].pin = 204;
    b->pins[18].gpio.mux[1].value = 0;

    strncpy(b->pins[19].name, "A5", 8);
    b->pins[19].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 1, 0 };
    b->pins[19].i2c.pinmap = 1;
    b->pins[19].i2c.mux_total = 2;
    b->pins[19].i2c.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[19].i2c.mux[0].pin = 213;
    b->pins[19].i2c.mux[0].value = MRAA_GPIO_IN;
    b->pins[19].i2c.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[19].i2c.mux[1].pin = 205;
    b->pins[19].i2c.mux[1].value = 0;
    b->pins[19].aio.pinmap = 5;
    b->pins[19].aio.mux_total = 2;
    b->pins[19].aio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[19].aio.mux[0].pin = 213;
    b->pins[19].aio.mux[0].value = MRAA_GPIO_IN;
    b->pins[19].aio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[19].aio.mux[1].pin = 205;
    b->pins[19].aio.mux[1].value = 1;
    b->pins[19].gpio.pinmap = 165;
    b->pins[19].gpio.mux_total = 2;
    b->pins[19].gpio.mux[0].pincmd = PINCMD_SET_DIRECTION;
    b->pins[19].gpio.mux[0].pin = 213;
    b->pins[19].gpio.mux[0].value = MRAA_GPIO_IN;
    b->pins[19].gpio.mux[1].pincmd = PINCMD_SET_OUT_VALUE;
    b->pins[19].gpio.mux[1].pin = 205;
    b->pins[19].gpio.mux[1].value = 0;

    // BUS DEFINITIONS
    b->i2c_bus_count = 9;
    b->def_i2c_bus = 6;
    int ici;
    for (ici = 0; ici < 9; ici++) {
        b->i2c_bus[ici].bus_id = -1;
    }
    b->i2c_bus[6].bus_id = 6;
    b->i2c_bus[6].sda = 18;
    b->i2c_bus[6].scl = 19;

    b->spi_bus_count = 1;
    b->def_spi_bus = 0;
    b->spi_bus[0].bus_id = 5;
    b->spi_bus[0].slave_s = 1;
    b->spi_bus[0].cs = 10;
    b->spi_bus[0].mosi = 11;
    b->spi_bus[0].miso = 12;
    b->spi_bus[0].sclk = 13;

    b->uart_dev_count = 1;
    b->def_uart_dev = 0;
    b->uart_dev[0].rx = 0;
    b->uart_dev[0].tx = 1;
    b->uart_dev[0].device_path = UART_DEV_PATH;

    //TODO we can't switch pinmode right now
    int il;
    for (il = 0; il < MRAA_INTEL_EDISON_PINCOUNT; il++) {
        pinmodes[il].gpio.sysfs = -1;
        pinmodes[il].gpio.mode = -1;
        pinmodes[il].pwm.sysfs = -1;
        pinmodes[il].pwm.mode = -1;
        pinmodes[il].i2c.sysfs = -1;
        pinmodes[il].i2c.mode = -1;
        pinmodes[il].spi.sysfs = -1;
        pinmodes[il].spi.mode = -1;
        pinmodes[il].uart.sysfs = -1;
        pinmodes[il].uart.mode = -1;
    }
    pinmodes[0].gpio.sysfs = 130;
    pinmodes[0].gpio.mode = 0;
    pinmodes[0].uart.sysfs = 130;
    pinmodes[0].uart.mode = 1;
    pinmodes[1].gpio.sysfs = 131;
    pinmodes[1].gpio.mode = 0;
    pinmodes[1].uart.sysfs = 131;
    pinmodes[1].uart.mode = 1;
    pinmodes[2].gpio.sysfs = 128;
    pinmodes[2].gpio.mode = 0;
    pinmodes[2].uart.sysfs = 128;
    pinmodes[2].uart.mode = 1;
    pinmodes[3].gpio.sysfs = 12;
    pinmodes[3].gpio.mode = 0;
    pinmodes[3].pwm.sysfs = 12;
    pinmodes[3].pwm.mode = 1;

    pinmodes[4].gpio.sysfs = 129;
    pinmodes[4].gpio.mode = 0;
    pinmodes[4].uart.sysfs = 129;
    pinmodes[4].uart.mode = 1;
    pinmodes[5].gpio.sysfs = 13;
    pinmodes[5].gpio.mode = 0;
    pinmodes[5].pwm.sysfs = 13;
    pinmodes[5].pwm.mode = 1;
    pinmodes[6].gpio.sysfs = 182;
    pinmodes[6].gpio.mode = 0;
    pinmodes[6].pwm.sysfs = 182;
    pinmodes[6].pwm.mode = 1;

    // 7 and 8 are provided by something on i2c, very simple

    pinmodes[9].gpio.sysfs = 183;
    pinmodes[9].gpio.mode = 0;
    pinmodes[9].pwm.sysfs = 183;
    pinmodes[9].pwm.mode = 1;

    pinmodes[10].gpio.sysfs = 41;
    pinmodes[10].gpio.mode = 0;
    pinmodes[10].spi.sysfs = 111; // Different pin provides, switched at mux level.
    pinmodes[10].spi.mode = 1;

    pinmodes[11].gpio.sysfs = 43;
    pinmodes[11].gpio.mode = 0;
    pinmodes[11].spi.sysfs = 115; // Different pin provides, switched at mux level.
    pinmodes[11].spi.mode = 1;

    pinmodes[12].gpio.sysfs = 42;
    pinmodes[12].gpio.mode = 0;
    pinmodes[12].spi.sysfs = 114; // Different pin provides, switched at mux level.
    pinmodes[12].spi.mode = 1;

    pinmodes[13].gpio.sysfs = 40;
    pinmodes[13].gpio.mode = 0;
    pinmodes[13].spi.sysfs = 109; // Different pin provides, switched at mux level.
    pinmodes[13].spi.mode = 1;

    // Everything else but A4 A5 LEAVE

    pinmodes[18].gpio.sysfs = 14;
    pinmodes[18].gpio.mode = 0;
    pinmodes[18].i2c.sysfs = 28;
    pinmodes[18].i2c.mode = 1;

    pinmodes[19].gpio.sysfs = 165;
    pinmodes[19].gpio.mode = 0;
    pinmodes[19].i2c.sysfs = 27;
    pinmodes[19].i2c.mode = 1;

    return b;
error:
    syslog(LOG_CRIT, "edison: Arduino board failed to initialise");
    free(b);
    return NULL;
}
