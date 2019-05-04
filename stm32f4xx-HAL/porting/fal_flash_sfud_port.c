/*
 * File      : fal_flash_sfud_port.c
 * This file is part of FAL (Flash Abstraction Layer) package
 * COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-26     armink       the first version
 */

#include <fal.h>
#include <sfud.h>

#ifdef FAL_FLASH_PORT_DRIVER_SFUD

#ifdef RT_USING_SFUD
#include <spi_flash_sfud.h>
#endif

#ifndef NOR_FLASH_SPI_DEV_NAME
#define NOR_FLASH_SPI_DEV_NAME             "norspi"
#endif

static int init(void);
static int read(long offset, uint8_t *buf, size_t size);
static int write(long offset, const uint8_t *buf, size_t size);
static int erase(long offset, size_t size);

static sfud_flash_t sfud_dev = RT_NULL;

static int init(void)
{

#ifdef RT_USING_SFUD
    /* RT-Thread RTOS platform */
	sfud_dev = rt_sfud_flash_find(NOR_FLASH_SPI_DEV_NAME);
#else
    /* bare metal platform */
    extern sfud_flash sfud_norflash;
    sfud_dev = &sfud_norflash;
#endif

    if (RT_NULL == sfud_dev)
        return -1;

    return 0;
}

static int read(long offset, uint8_t *buf, size_t size)
{
	if ((sfud_dev == RT_NULL) || (sfud_dev->init_ok == 0))
		return -1;
	
    sfud_read(sfud_dev, nor_flash.addr + offset, size, buf);

    return size;
}

static int write(long offset, const uint8_t *buf, size_t size)
{
    if ((sfud_dev == RT_NULL) || (sfud_dev->init_ok == 0))
		return -1;
	
    if (sfud_write(sfud_dev, nor_flash.addr + offset, size, buf) != SFUD_SUCCESS)
        return -1;

    return size;
}

static int erase(long offset, size_t size)
{
    if ((sfud_dev == RT_NULL) || (sfud_dev->init_ok == 0))
		return -1;
	
    if (sfud_erase(sfud_dev, nor_flash.addr + offset, size) != SFUD_SUCCESS)
        return -1;

    return size;
}

const struct fal_flash_dev nor_flash = {"nor_flash", 0, 16 * 1024 * 1024, 4096, {init, read, write, erase}};
#endif /* FAL_FLASH_PORT_DRIVER_SFUD */
