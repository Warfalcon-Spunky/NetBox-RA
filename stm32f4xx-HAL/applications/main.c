/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-07-29     Arda.Fu      first implementation
 */
#include <rtthread.h>
#include <board.h>

#ifdef RT_LWIP_PPP
#include "netif/ppp/ppp_opts.h"
#include "lwip/sio.h"
#include "lwip/dns.h"
#include "ppp/pppapi.h"
#include "ppp/pppos.h"
#include "ppp/ppp.h"
#include "ppp/ppp_impl.h"
#endif /* RT_LWIP_PPP */
#include "netif/ethernetif.h"

#define LOG_TAG              "main"
#include <at_log.h>

int main(void)
{
    /* user app entry */

    return 0;
}

extern void sim8000c_set_up(void);
extern void sim8000c_set_down(void);
extern void ethernet_set_up(void);
extern void ethernet_set_down(void);


int network_set_up(int argc, char **argv)
{
	if (argc < 2)
	{
		LOG_I("the parameter must be least 2");
		return -1;
	}

	if (!rt_strcmp(argv[1], "eth"))
	{
		sim8000c_set_down();
		ethernet_set_up();
	}
	else if (!rt_strcmp(argv[1], "ppp"))
	{
		ethernet_set_down();
		sim8000c_set_up();		
	}
    
    return 0;
}
#ifdef FINSH_USING_MSH
#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(network_set_up, set_up, chosse network);
#endif


