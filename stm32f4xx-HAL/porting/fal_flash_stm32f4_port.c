/*
 * File      : fal_flash_stm32f2_port.c
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

#ifdef FAL_FLASH_PORT_DRIVER_STM32F4
#include <stm32f4xx_hal.h>
#include "board.h"

#define LOG_TAG     "flash.onchip"
#define LOG_LVL     LOG_LVL_DBG
#include <at_log.h>

#define FLASH_START_ADRESS     ((uint32_t)0x08000000)
#define FLASH_END_ADDRESS      ((uint32_t)(0x08000000 + 1024 * 1024))

/* base address of the flash sectors */
#define ADDR_FLASH_SECTOR_0      ((uint32_t)0x08000000) /* Base address of Sector 0, 16 K bytes   */
#define ADDR_FLASH_SECTOR_1      ((uint32_t)0x08004000) /* Base address of Sector 1, 16 K bytes   */
#define ADDR_FLASH_SECTOR_2      ((uint32_t)0x08008000) /* Base address of Sector 2, 16 K bytes   */
#define ADDR_FLASH_SECTOR_3      ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 K bytes   */
#define ADDR_FLASH_SECTOR_4      ((uint32_t)0x08010000) /* Base address of Sector 4, 64 K bytes   */
#define ADDR_FLASH_SECTOR_5      ((uint32_t)0x08020000) /* Base address of Sector 5, 128 K bytes  */
#define ADDR_FLASH_SECTOR_6      ((uint32_t)0x08040000) /* Base address of Sector 6, 128 K bytes  */
#define ADDR_FLASH_SECTOR_7      ((uint32_t)0x08060000) /* Base address of Sector 7, 128 K bytes  */
#define ADDR_FLASH_SECTOR_8      ((uint32_t)0x08080000) /* Base address of Sector 8, 128 K bytes  */
#define ADDR_FLASH_SECTOR_9      ((uint32_t)0x080A0000) /* Base address of Sector 9, 128 K bytes  */
#define ADDR_FLASH_SECTOR_10     ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 K bytes */
#define ADDR_FLASH_SECTOR_11     ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 K bytes */

/**
 * Get the sector of a given address
 *
 * @param address flash address
 *
 * @return The sector of a given address
 */
static uint32_t stm32_get_sector(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}

/**
 * Get the sector size
 *
 * @param sector sector
 *
 * @return sector size
 */
static uint32_t stm32_get_sector_size(uint32_t sector) 
{
	uint32_t sector_size = 0;
		
	RT_ASSERT(sector < FLASH_SECTOR_TOTAL);

    switch (sector) 
	{
    case FLASH_SECTOR_0: 
		sector_size = 16 * 1024;
		break;
    case FLASH_SECTOR_1: 
		sector_size = 16 * 1024;
		break;
    case FLASH_SECTOR_2: 
		sector_size = 16 * 1024;
		break;
    case FLASH_SECTOR_3: 
		sector_size = 16 * 1024;
		break;
    case FLASH_SECTOR_4: 
		sector_size = 64 * 1024;
		break;
    case FLASH_SECTOR_5: 
		sector_size = 128 * 1024;
		break;
    case FLASH_SECTOR_6: 
		sector_size = 128 * 1024;
		break;
    case FLASH_SECTOR_7: 
		sector_size = 128 * 1024;
		break;
    case FLASH_SECTOR_8: 
		sector_size = 128 * 1024;
		break;
    case FLASH_SECTOR_9: 
		sector_size = 128 * 1024;
		break;
    case FLASH_SECTOR_10: 
		sector_size = 128 * 1024;
		break;
    case FLASH_SECTOR_11: 
		sector_size = 128 * 1024;
		break;
    default: 
		break;
    }
	
	return sector_size;
}

static int read(long offset, uint8_t *buf, size_t size)
{
    size_t i;
    uint32_t addr = stm32f4_onchip_flash.addr + offset;

	if ((addr + size) > FLASH_END_ADDRESS)
    {
        LOG_E("ERROR: read outrange flash size! addr is (0x%p)\n", (void*)(addr + size));
        return -1;
    }
	
    for (i = 0; i < size; i++, addr++, buf++)
        *buf = *(uint8_t *) addr;

    return size;
}

static int write(long offset, const uint8_t *buf, size_t size)
{
	size_t i, j;
	int8_t status = 0;
	uint64_t write_data = 0, temp_data = 0;
	uint32_t addr = stm32f4_onchip_flash.addr + offset;

	if ((addr + size) > FLASH_END_ADDRESS)
	{
		LOG_D("ERROR: write outrange flash size! addr is (0x%p)\n", (void*)(addr + size));
		return -1;
	}

	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);

	if (size < 1)
		return -1;

	for (i = 0; i < size;)
	{
		if ((size - i) < 8)
		{
			for (j = 0; (size - i) > 0; i++, j++)
			{
				temp_data = *buf;
				write_data = (write_data) | (temp_data << 8 * j);
				buf++;
			}
		}
		else
		{
			for (j = 0; j < 8; j++, i++)
			{
				temp_data = *buf;
				write_data = (write_data) | (temp_data << 8 * j);
				buf++;
			}
		}

		/* write data */
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, write_data) == HAL_OK)
		{
			/* Check the written value */
			if (*(uint64_t*)addr != write_data)
			{
				LOG_D("ERROR: write data != read data\n");
				status = -1;
				goto __write_exit;
			}
		}
		else
		{
			status = -1;
			goto __write_exit;
		}
		
		temp_data = 0;
		write_data = 0;

		addr +=8;
	}

__write_exit:
	HAL_FLASH_Lock();
	
	if (status != 0)
		return status;

	return size;

}

static int erase(long offset, size_t size)
{
    size_t erased_size = 0;
    uint32_t cur_erase_sector;
	uint32_t sector_err;
	HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef erase_init;
	
    uint32_t addr = stm32f4_onchip_flash.addr + offset;

	if ((addr + size) > FLASH_END_ADDRESS)
    {
        LOG_D("ERROR: erase outrange flash size! addr is (0x%p)\n", (void*)(addr + size));
        return -1;
    }

	erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
	erase_init.Banks        = FLASH_BANK_1;
	erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	erase_init.NbSectors    = 1;

    /* start erase */
	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);

    /* it will stop when erased size is greater than setting size */
    while (erased_size < size)
    {    	
        cur_erase_sector = stm32_get_sector(addr + erased_size);	
		
		erase_init.Sector = cur_erase_sector;		
		status = HAL_FLASHEx_Erase(&erase_init, &sector_err);
		if (status != HAL_OK)
            goto __erase_exit;
		
        erased_size += stm32_get_sector_size(cur_erase_sector);
    }
__erase_exit:	
    HAL_FLASH_Lock();

    return size;
}

const struct fal_flash_dev stm32f4_onchip_flash = {"onchip_flash", FLASH_START_ADRESS, 1024 * 1024, 128 * 1024, {RT_NULL, read, write, erase}};

#endif
