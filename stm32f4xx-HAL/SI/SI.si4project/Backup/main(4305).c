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

#define LOG_TAG              "ppps"
#include <at_log.h>

int main(void)
{
    /* user app entry */

    return 0;
}

#define PPP_USERNAME  ""
#define PPP_PASSWORD  ""

static ppp_pcb *ppp = NULL;
static struct netif ppp_netif;
static uint8_t ppp_rx_buff[1366]; 
static uint32_t ppp_rx_cnt;

static void pppLinkStatusCallback(ppp_pcb *pcb, int errCode, void *ctx)
{
    struct netif *pppif = ppp_netif(pcb);
	
    LWIP_UNUSED_ARG(ctx);

    switch(errCode) {
    case PPPERR_NONE: {             /* No error. */
        LOG_E("pppLinkStatusCallback: PPPERR_NONE\n");
        #if LWIP_IPV4
        LOG_E("   our_ipaddr  = %s\n", ip4addr_ntoa(netif_ip4_addr(pppif)));
        LOG_E("   his_ipaddr  = %s\n", ip4addr_ntoa(netif_ip4_gw(pppif)));
        LOG_E("   netmask     = %s\n", ip4addr_ntoa(netif_ip4_netmask(pppif)));
        #endif /* LWIP_IPV4 */
        #if LWIP_DNS
        LOG_E("   dns1        = %s\n", (char *)ipaddr_ntoa(dns_getserver(0)));
        LOG_E("   dns2        = %s\n", (char *)ipaddr_ntoa(dns_getserver(1)));
        #endif /* LWIP_DNS */

        break;
        }
    case PPPERR_PARAM: {           /* Invalid parameter. */
        LOG_E("pppLinkStatusCallback: PPPERR_PARAM\n");
        break;
        }
    case PPPERR_OPEN: {            /* Unable to open PPP session. */
        LOG_E("pppLinkStatusCallback: PPPERR_OPEN\n");
        break;
        }
    case PPPERR_DEVICE: {          /* Invalid I/O device for PPP. */
        LOG_E("pppLinkStatusCallback: PPPERR_DEVICE\n");
        break;
        }
    case PPPERR_ALLOC: {           /* Unable to allocate resources. */
        LOG_E("pppLinkStatusCallback: PPPERR_ALLOC\n");
        break;
        }
    case PPPERR_USER: {            /* User interrupt. */
        LOG_E("pppLinkStatusCallback: PPPERR_USER\n");
        break;
        }
    case PPPERR_CONNECT: {         /* Connection lost. */
        LOG_E("pppLinkStatusCallback: PPPERR_CONNECT\n");
        break;
    }
    case PPPERR_AUTHFAIL: {        /* Failed authentication challenge. */
        LOG_E("pppLinkStatusCallback: PPPERR_AUTHFAIL\n");
        break;
        }
    case PPPERR_PROTOCOL: {        /* Failed to meet protocol. */
        LOG_E("pppLinkStatusCallback: PPPERR_PROTOCOL\n");
        break;
    }
    case PPPERR_PEERDEAD: {        /* Connection timeout */
        LOG_E("pppLinkStatusCallback: PPPERR_PEERDEAD\n");
        break;
        }
    case PPPERR_IDLETIMEOUT: {     /* Idle Timeout */
        LOG_E("pppLinkStatusCallback: PPPERR_IDLETIMEOUT\n");
        break;
    }
    case PPPERR_CONNECTTIME: {     /* Max connect time reached */
        LOG_E("pppLinkStatusCallback: PPPERR_CONNECTTIME\n");
        break;
        }
    case PPPERR_LOOPBACK: {        /* Loopback detected */
        LOG_E("pppLinkStatusCallback: PPPERR_LOOPBACK\n");
        break;
        }
    default: {
        LOG_E("pppLinkStatusCallback: unknown errCode %d\n", errCode);
        break;
        }
    }

    if (errCode == PPPERR_NONE) {
        return;
    }

    if (errCode == PPPERR_USER) {
		ppp_free(pcb);
        return;
    }

    /* reconnect */
    ppp_connect(pcb, 30);
}

uint8_t suspend_t = 0;
static u32_t ppp_output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
	rt_device_t sio_dev;
    
    if (suspend_t == 2)
        return 0;
	
	sio_dev = rt_device_find("uart3");
	if (sio_dev == RT_NULL)
		return 0;
  
  	return sio_write((void *)sio_dev, data, len);
}


extern uint8_t dial;

static rt_err_t (*client_odev_rx_ind)(rt_device_t dev, rt_size_t size) = RT_NULL;

static void ppp_thread_entry(void *parameter)
{
	rt_device_t sio_dev;

	sio_dev = rt_device_find("uart3");
	if (sio_dev == RT_NULL)
		return;
	
	ppp = pppos_create(&ppp_netif, ppp_output_cb, pppLinkStatusCallback, NULL);
	if (ppp == RT_NULL)
		return;

	while (dial == 0)
	{
		rt_thread_delay(RT_TICK_PER_SECOND);
	}
    
    client_odev_rx_ind = sio_dev->rx_indicate;
    rt_device_set_rx_indicate(sio_dev, RT_NULL);
        
    ppp_set_default(ppp);
        
    ppp_set_usepeerdns(ppp, 1);
        
    ppp_set_auth(ppp, PPPAUTHTYPE_ANY, "", "");
    ppp_connect(ppp, 0);
    
	while (1)
	{
        if (suspend_t == 0)
        {
            ppp_rx_cnt = sio_read((void *)sio_dev, ppp_rx_buff, sizeof(ppp_rx_buff));
            if (ppp_rx_cnt)
                pppos_input_tcpip(ppp, ppp_rx_buff, ppp_rx_cnt);
        }

		rt_thread_delay(RT_TICK_PER_SECOND / 100);
	}
}

static void ppp_pause(int argc, char **argv)
{
    rt_device_t sio_dev;

	sio_dev = rt_device_find("uart3");
	if (sio_dev != RT_NULL)
	{
        suspend_t = 2;        
        rt_thread_delay(rt_tick_from_millisecond(1250));
        rt_device_write(sio_dev, 0, "+++", 3);     
        rt_thread_delay(rt_tick_from_millisecond(1250)); 
    }    
}
MSH_CMD_EXPORT(ppp_pause, ppp process);

static void ppp_run(int argc, char **argv)
{
	struct eth_device *eth_dev;
	
    eth_dev = (struct eth_device *)rt_device_find("e0");
	netif_set_default(eth_dev->netif);
}
MSH_CMD_EXPORT(ppp_run, ppp process);

static void suspend_ppp(int argc, char **argv)
{	
    ppp_close(ppp, 1);// second parameter is nocarrior = 0 关载波需要重新拨号
    suspend_t = 1; 
}

MSH_CMD_EXPORT(suspend_ppp, suspend ppp process);

static void resume_ppp(int argc, char **argv)
{	
    ppp = pppos_create(&ppp_netif, ppp_output_cb, pppLinkStatusCallback, NULL);
	if (ppp == RT_NULL)
		return;
    
    ppp_set_default(ppp);
    ppp_set_usepeerdns(ppp, 1);        
    ppp_set_auth(ppp, PPPAUTHTYPE_ANY, "", "");
    ppp_connect(ppp, 0);    
    suspend_t = 0; 
}

MSH_CMD_EXPORT(resume_ppp, resume ppp process);


int rt_ppp_init(void)
{
	rt_thread_t tid;
	
	/* create ppps thread */        
    tid = rt_thread_create("ppps", ppp_thread_entry, RT_NULL, 8192, RT_THREAD_PRIORITY_MAX - 24, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    
    return 0;
}
INIT_APP_EXPORT(rt_ppp_init);

