/*
 * Copyright (c) 2006-2018 RT-Thread Development Team. All rights reserved.
 * License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "iot_import.h"
#include "iot_export.h"

#include "rtthread.h"
#include "cJSON.h"
#include "easyflash.h"
#include "mqtt-def.h"


#define LOG_TAG              "ali-mqtt"       /* 该模块对应的标签。不定义时，默认：NO_TAG */
#define LOG_LVL              LOG_LVL_DBG     /* 该模块对应的日志输出级别。不定义时，默认：调试级别 */
#include <ulog.h>                            /* 必须在 LOG_TAG 与 LOG_LVL 下面 */

#define MQTT_MSGLEN                             (1024)
#define MQTT_KEEPALIVE_INTERNAL                 (120)

static rt_mailbox_t device_mailbox;
static void *mqtt_device_hd = RT_NULL;
static uint8_t mqtt_is_running = 0;

static void ali_mqtt_property_set_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg);
static void ali_mqtt_door_ctrl_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg);
static void ali_mqtt_device_ctrl_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg);
static void ali_mqtt_timeout_alarm_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg);
static void ali_mqtt_device_error_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg);
static void ali_mqtt_property_post_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg);
static void ali_mqtt_device_info_update_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg);

typedef struct
{
    const char *topic_filter;
    iotx_mqtt_qos_t qos;
    iotx_mqtt_event_handle_func_fpt topic_handle_func;
    void *pcontext;
} mqtt_subscribe_item, *mqtt_subscribe_item_t;

static const mqtt_subscribe_item mqtt_sub_item[] = 
{
    {ALI_PROPERTY_SET_SUB,                 IOTX_MQTT_QOS1, ali_mqtt_property_set_msg_arrive,       RT_NULL},    
    {ALI_SERVICE_DOOR_CTRL_SUB,            IOTX_MQTT_QOS1, ali_mqtt_door_ctrl_msg_arrive,          RT_NULL},
    {ALI_SERVICE_DEVICE_CTRL_SUB,          IOTX_MQTT_QOS1, ali_mqtt_device_ctrl_msg_arrive,        RT_NULL},
    {ALI_EVENT_TIMEOUT_ALARM_REPLY_SUB,    IOTX_MQTT_QOS1, ali_mqtt_timeout_alarm_msg_arrive,      RT_NULL},
    {ALI_EVENT_DEVICE_ERROR_REPLY_SUB,     IOTX_MQTT_QOS1, ali_mqtt_device_error_msg_arrive,       RT_NULL},
    {ALI_PROPERTY_POST_REPLY_SUB,          IOTX_MQTT_QOS1, ali_mqtt_property_post_msg_arrive,      RT_NULL},
    {ALI_DEVICEINFO_UPDATE_REPLY_SUB,      IOTX_MQTT_QOS1, ali_mqtt_device_info_update_msg_arrive, RT_NULL}
};

static void ali_mqtt_event_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_pt topic_info = (iotx_mqtt_topic_info_pt)msg->msg;
    if (topic_info == NULL)
    {
        rt_kprintf("Topic info is null! Exit.");
        return;
    }
    uintptr_t packet_id = (uintptr_t)topic_info;

    switch (msg->event_type) 
    {
        case IOTX_MQTT_EVENT_UNDEF:
            LOG_D("undefined event occur.");
            break;

        case IOTX_MQTT_EVENT_DISCONNECT:
            LOG_D("MQTT disconnect.");
            break;
        case IOTX_MQTT_EVENT_RECONNECT:
            LOG_D("MQTT reconnect.");
            break;

        case IOTX_MQTT_EVENT_SUBCRIBE_SUCCESS:
            LOG_D("subscribe success, packet-id=%u", (unsigned int)packet_id);
            break;
        case IOTX_MQTT_EVENT_SUBCRIBE_TIMEOUT:
            LOG_D("subscribe wait ack timeout, packet-id=%u", (unsigned int)packet_id);
            break;
        case IOTX_MQTT_EVENT_SUBCRIBE_NACK:
            LOG_D("subscribe nack, packet-id=%u", (unsigned int)packet_id);
            break;

        case IOTX_MQTT_EVENT_UNSUBCRIBE_SUCCESS:
            LOG_D("unsubscribe success, packet-id=%u", (unsigned int)packet_id);
            break;
        case IOTX_MQTT_EVENT_UNSUBCRIBE_TIMEOUT:
            LOG_D("unsubscribe timeout, packet-id=%u", (unsigned int)packet_id);
            break;
        case IOTX_MQTT_EVENT_UNSUBCRIBE_NACK:
            LOG_D("unsubscribe nack, packet-id=%u", (unsigned int)packet_id);
            break;
        case IOTX_MQTT_EVENT_PUBLISH_SUCCESS:
            LOG_D("publish success, packet-id=%u", (unsigned int)packet_id);
            break;
        case IOTX_MQTT_EVENT_PUBLISH_TIMEOUT:
            LOG_D("publish timeout, packet-id=%u", (unsigned int)packet_id);
            break;
        case IOTX_MQTT_EVENT_PUBLISH_NACK:
            LOG_D("publish nack, packet-id=%u", (unsigned int)packet_id);
            break;
        case IOTX_MQTT_EVENT_PUBLISH_RECVEIVED:
            LOG_D("topic message arrived but without any related handle: topic=%.*s, topic_msg=%.*s",
                          topic_info->topic_len,
                          topic_info->ptopic,
                          topic_info->payload_len,
                          topic_info->payload);
            break;

        case IOTX_MQTT_EVENT_BUFFER_OVERFLOW:
            LOG_D("buffer overflow, %s", msg->msg);
            break;

        default:
            LOG_D("Should NOT arrive here.");
            break;
    }
}

rt_err_t mqtt_post_mbox(rt_mailbox_t mailbox, char *msg_str, rt_int32_t timeout)
{
    if ((mailbox == RT_NULL) || (msg_str == RT_NULL))
        return -RT_ERROR;
    
    char *mbox_msg_str = rt_malloc(rt_strlen(msg_str) + 1);
    if (mbox_msg_str == RT_NULL)
        return -RT_ENOMEM; 

    rt_memset(mbox_msg_str, 0, rt_strlen(msg_str) + 1);
    rt_strncpy(mbox_msg_str, msg_str, rt_strlen(msg_str));

    return rt_mb_send_wait(mailbox, (rt_uint32_t)mbox_msg_str, timeout);
}

rt_err_t mqtt_pend_mbox(rt_mailbox_t mailbox, char *msg_str, uint16_t msg_str_len, rt_int32_t timeout)
{
    if ((mailbox == RT_NULL) || (msg_str == RT_NULL))
        return -RT_ERROR;

    char *mbox_msg_str = RT_NULL;

    if (rt_mb_recv(mailbox, (rt_ubase_t *)&mbox_msg_str, timeout) == RT_EOK)
    {
        if (mbox_msg_str != RT_NULL)
        {
            rt_strncpy(msg_str, mbox_msg_str, msg_str_len);
            rt_free(mbox_msg_str);
            return RT_EOK;
        }
    }
    return -RT_ERROR;
}

static void mqtt_asyn_reply_pub(void *handle, const char *topic_idx, char *id, char *code, char *data)
{    
	char *msg_pub = RT_NULL;
	char *topic_name = RT_NULL;
	iotx_mqtt_topic_info_pt topic_msg = RT_NULL;

	if (handle == RT_NULL)
	{
		LOG_W("IOT handler is null.");
		goto __asyn_pub_exit;
	}
	
	msg_pub = (char *)rt_malloc(128);
    if (msg_pub == RT_NULL)
    {	
    	LOG_W("not enough memory for msg pub buffer.");
		goto __asyn_pub_exit;
    }	
	rt_memset(msg_pub, 0, 128);
	rt_snprintf(msg_pub, 128, "{\"id\": \"%s\",\"code\": \"%s\",\"data\": {%s}}", id, code, data);

	topic_msg = (iotx_mqtt_topic_info_pt)rt_malloc(sizeof(iotx_mqtt_topic_info_t));
	if (topic_msg == RT_NULL)
    {	
    	LOG_W("not enough memory for topic msg buffer.");
		goto __asyn_pub_exit;
    }
    rt_memset(&topic_msg, 0, sizeof(iotx_mqtt_topic_info_t));
    topic_msg->qos = IOTX_MQTT_QOS1;
    topic_msg->retain = 0;
    topic_msg->dup = 0;
    topic_msg->payload = (void *)msg_pub;
    topic_msg->payload_len = rt_strlen(msg_pub);

	topic_name = (char *)rt_malloc(128);
	if (topic_name == RT_NULL)
    {	
    	LOG_W("not enough memory for topic name buffer.");
		goto __asyn_pub_exit;
    }
	rt_memset(topic_name, 0, 128);
	if (ef_get_env_blob(topic_idx, topic_name, 127, RT_NULL) > 0)
    	IOT_MQTT_Publish(handle, topic_name, topic_msg);
	
__asyn_pub_exit:
	if (msg_pub)
		rt_free(msg_pub);
	if (topic_msg)
		rt_free(topic_msg);
	if (topic_name)
		rt_free(topic_name);
}

static void ali_mqtt_property_set_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg)
{
    uint8_t op_mode = 0;
	size_t para_len;
    cJSON *root, *id, *params, *mode, *para_idx, *para_val;
	iotx_mqtt_topic_info_pt ptopic_info = (iotx_mqtt_topic_info_pt) msg->msg;

    LOG_D("subcrible message arrive: %.s.", ptopic_info->topic_len, ptopic_info->ptopic);

	root = cJSON_Parse(ptopic_info->payload);
    if (root == RT_NULL)
    {
        LOG_D("cJSON parse failed.");
        goto __property_set_exit;
    }

	id = cJSON_GetObjectItem(root, "id");
    if (id == RT_NULL)
    {
        LOG_D("cJSON get object[id] failed.");
        goto __property_set_exit;
    }
    params = cJSON_GetObjectItem(root, "params");
    if (params == RT_NULL)
    {
        LOG_D("cJSON get object[params] failed.");
        goto __property_set_exit;
    }

	mode = cJSON_GetObjectItem(params, "mode");
    if (mode == RT_NULL)
    {
        LOG_D("cJSON get object[mode] failed.");
        goto __property_set_exit;
    }
	para_idx = cJSON_GetObjectItem(params, "para_idx");
    if (para_idx == RT_NULL)
    {
        LOG_D("cJSON get object[para_idx] failed.");
        goto __property_set_exit;
    }
	para_val = cJSON_GetObjectItem(params, "para_val");
    if (para_val == RT_NULL)
    {
        LOG_D("cJSON get object[para_val] failed.");
        goto __property_set_exit;
    }
    
	para_len = rt_strlen(mode->valuestring);
	
	if (!rt_strncmp("write", mode->valuestring, para_len))
		op_mode = op_mode | 0x01;
	else if (!rt_strncmp("read", mode->valuestring, para_len))
		op_mode = op_mode | 0x02;
	else
	{
		LOG_D("KV operation mode error.");
        goto __property_set_exit;
	}

	ef_get_env_blob(para_idx->valuestring, RT_NULL, 0, &para_len);
	if (para_len > 0)
	{
		if (op_mode & 0x01)
		{
			ef_set_env_blob(para_idx->valuestring, para_val->valuestring, rt_strlen(para_val->valuestring));
			mqtt_asyn_reply_pub(mqtt_device_hd, ALI_PROPERTY_SET_REPLY_PUB, id->valuestring, ALI_CODE_OK, "");
		}
		else
		{
			char *para_val_buff = rt_malloc(128);
			if (para_val_buff)
			{
				rt_memset(para_val_buff, 0, 128);
				rt_strncpy(para_val_buff, "para_val: ", 128);
				ef_get_env_blob(para_idx->valuestring, para_val_buff + rt_strlen(para_val_buff), 127 - rt_strlen(para_val_buff), RT_NULL);
				mqtt_asyn_reply_pub(mqtt_device_hd, ALI_PROPERTY_SET_REPLY_PUB, id->valuestring, ALI_CODE_OK, para_val_buff);
				rt_free(para_val_buff);
			}
			else
				mqtt_asyn_reply_pub(mqtt_device_hd, ALI_PROPERTY_SET_REPLY_PUB, id->valuestring, ALI_CODE_WRITE_IDX_ERROR, "");
		}
	}
	else
		mqtt_asyn_reply_pub(mqtt_device_hd, ALI_PROPERTY_SET_REPLY_PUB, id->valuestring, ALI_CODE_WRITE_IDX_ERROR, "");

__property_set_exit:
	if (root)
        cJSON_Delete(root);
}

static void ali_mqtt_door_ctrl_msg_arrive (void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg)
{
    cJSON *root, *id, *params, *door_idx;
    iotx_mqtt_topic_info_pt ptopic_info = (iotx_mqtt_topic_info_pt) msg->msg;

    LOG_D("subcrible message arrive: %.s.", ptopic_info->topic_len, ptopic_info->ptopic);

    root = cJSON_Parse(ptopic_info->payload);
    if (root == RT_NULL)
    {
        LOG_D("cJSON parse failed.");
        goto __door_ctrl_exit;
    }

	id = cJSON_GetObjectItem(root, "id");
    if (id == RT_NULL)
    {
        LOG_D("cJSON get object[id] failed.");
        goto __door_ctrl_exit;
    }

    params = cJSON_GetObjectItem(root, "params");
    if (params == RT_NULL)
    {
        LOG_D("cJSON get object[params] failed.");
        goto __door_ctrl_exit;
    }

	door_idx = cJSON_GetObjectItem(params, "door_idx");
    if (door_idx == RT_NULL)
    {
        LOG_D("cJSON get object[door_idx] failed.");
        goto __door_ctrl_exit;
    }

    LOG_D("post mailbox: %.s.", rt_strlen(door_idx->valuestring), door_idx->valuestring);
    /* post mailbox for exectue thread */
//    mqtt_post_mbox(device_mailbox, door_idx->valuestring, rt_tick_from_millisecond(1000));

	char msg_dat[64];
	rt_snprintf(msg_dat, sizeof(msg_dat), "door_idx: %s", door_idx->valuestring);
	mqtt_asyn_reply_pub(mqtt_device_hd, ALI_SERVICE_DOOR_CTRL_REPLY_PUB, id->valuestring, ALI_CODE_OK, msg_dat);
	
__door_ctrl_exit:
    if (root)
        cJSON_Delete(root);
}

static void ali_mqtt_device_ctrl_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg)
{
    cJSON *root, *id, *params, *ctrl_cmd, *ctrl_para;
    iotx_mqtt_topic_info_pt ptopic_info = (iotx_mqtt_topic_info_pt) msg->msg;

    LOG_D("subcrible message arrive: %.s.", ptopic_info->topic_len, ptopic_info->ptopic);

    root = cJSON_Parse(ptopic_info->payload);
    if (root == RT_NULL)
    {
        LOG_D("cJSON parse failed.");
        goto __device_ctrl_exit;
    }

	id = cJSON_GetObjectItem(root, "id");
    if (id == RT_NULL)
    {
        LOG_D("cJSON get object[id] failed.");
        goto __device_ctrl_exit;
    }

    params = cJSON_GetObjectItem(root, "params");
    if (params == RT_NULL)
    {
        LOG_D("cJSON get object[params] failed.");
        goto __device_ctrl_exit;
    }

	ctrl_cmd = cJSON_GetObjectItem(params, "ctrl_cmd");
    if (ctrl_cmd == RT_NULL)
    {
        LOG_D("cJSON get object[ctrl_cmd] failed.");
        goto __device_ctrl_exit;
    }

	ctrl_para = cJSON_GetObjectItem(params, "ctrl_para");
    if (ctrl_para == RT_NULL)
    {
        LOG_D("cJSON get object[ctrl_para] failed.");
        goto __device_ctrl_exit;
    }

	if (!rt_strncmp("radiation", ctrl_para->valuestring, rt_strlen(ctrl_para->valuestring)))
	{
		if (!rt_strncmp("reboot", ctrl_cmd->valuestring, rt_strlen(ctrl_cmd->valuestring)))
		{
			LOG_D("remote command: reboot");
			mqtt_asyn_reply_pub(mqtt_device_hd, ALI_SERVICE_DEVICE_CTRL_REPLY_PUB, id->valuestring, ALI_CODE_OK, "");
		}
		else if (!rt_strncmp("shutdown", ctrl_cmd->valuestring, rt_strlen(ctrl_cmd->valuestring)))
		{
			LOG_D("remote command: shutdown");
			mqtt_asyn_reply_pub(mqtt_device_hd, ALI_SERVICE_DEVICE_CTRL_REPLY_PUB, id->valuestring, ALI_CODE_OK, "");
		}
		else
		{
			LOG_D("No such remote command defined: %.s.", rt_strlen(ctrl_cmd->valuestring), ctrl_cmd->valuestring);
			mqtt_asyn_reply_pub(mqtt_device_hd, ALI_SERVICE_DEVICE_CTRL_REPLY_PUB, id->valuestring, ALI_CODE_DEV_CTRL_FAIL, "");
		}
	}
	else
	{
		LOG_D("No such remote parament defined: %.s.", rt_strlen(ctrl_cmd->valuestring), ctrl_para->valuestring);
		mqtt_asyn_reply_pub(mqtt_device_hd, ALI_SERVICE_DEVICE_CTRL_REPLY_PUB, id->valuestring, ALI_CODE_DEV_CTRL_FAIL, "");
	}	
    
__device_ctrl_exit:
    if (root)
        cJSON_Delete(root);
}

static void ali_mqtt_timeout_alarm_msg_arrive(void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg)
{
	LOG_I("-------------------");
    LOG_I("timeout alarm feedback.");
    LOG_I("-------------------");
}

static void ali_mqtt_device_error_msg_arrive (void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg)
{
    LOG_I("-------------------");
    LOG_I("device error feedback.");
    LOG_I("-------------------");
}

static void ali_mqtt_property_post_msg_arrive (void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg)
{
    LOG_I("-------------------");
    LOG_I("property post feedback.");
    LOG_I("-------------------");
}

static void ali_mqtt_device_info_update_msg_arrive (void *pcontext, void *handle, iotx_mqtt_event_msg_pt msg)
{
    LOG_I("-------------------");
    LOG_I("device info update feedback.");
    LOG_I("-------------------");
}

static void mqtt_device(void *arg)
{
	int i;
	char *topic_name = RT_NULL, *msg_writebuf = RT_NULL, *msg_readbuf = RT_NULL;
    char product_key[PRODUCT_KEY_LEN + 1], device_name[DEVICE_NAME_LEN + 1], device_secret[DEVICE_SECRET_LEN + 1];

    iotx_mqtt_param_t device_params;
    iotx_conn_info_pt mqtt_conn_info = RT_NULL; 

	/* get the secret of ali-iot platform */
    HAL_GetProductKey(product_key);
    HAL_GetDeviceName(device_name);
    HAL_GetDeviceSecret(device_secret);

    while (1)
    {        
        /* connect ali-iot platform */
        if (0 != IOT_SetupConnInfo(product_key, device_name, device_secret, (void **)&mqtt_conn_info)) 
        {
            LOG_I("platform authentication request failed!");
            goto do_device_exit;
        }

        /* malloc write and read buffer of MQTT */
        if ((RT_NULL == (msg_writebuf = (char *)rt_malloc(MQTT_MSGLEN))) 
        || (RT_NULL == (msg_readbuf = (char *)rt_malloc(MQTT_MSGLEN))))
        {
            LOG_W("not enough memory.");
            goto do_device_exit;
        }

        /* Initialize MQTT parameter */
        rt_memset(&device_params, 0x0, sizeof(device_params));
        /* feedback parameter of platform when use IOT_SetupConnInfo() connect */
        device_params.port      = mqtt_conn_info->port;
        device_params.host      = mqtt_conn_info->host_name;
        device_params.client_id = mqtt_conn_info->client_id;
        device_params.username  = mqtt_conn_info->username;
        device_params.password  = mqtt_conn_info->password;
        device_params.pub_key   = mqtt_conn_info->pub_key;
        /* timeout of request. uint: ms */
        device_params.request_timeout_ms    = 2000;
        device_params.clean_session         = 0;
        /* internal of keepalive checking: 60s~300s */
        device_params.keepalive_interval_ms = MQTT_KEEPALIVE_INTERNAL * 1000; 
        device_params.pread_buf             = msg_readbuf;
        device_params.read_buf_size         = MQTT_MSGLEN;
        device_params.pwrite_buf            = msg_writebuf;
        device_params.write_buf_size        = MQTT_MSGLEN;
        /* configure handle of event */
        device_params.handle_event.h_fp     = ali_mqtt_event_handle;
        device_params.handle_event.pcontext = NULL;

        /* construct a MQTT device with specify parameter */
        mqtt_device_hd = IOT_MQTT_Construct(&device_params);
        if (RT_NULL == mqtt_device_hd) 
        {
            LOG_D("construct MQTT failed!");
            goto do_device_exit;
        }

		topic_name = rt_malloc(128);
		if (topic_name == RT_NULL)
		{
			LOG_D("not enough memory for topic name!");
            goto do_device_exit;
		}
        /* sbuscribe all topic */
        for (i = 0; i < sizeof(mqtt_sub_item) / sizeof(mqtt_subscribe_item); i++)
        {
        	rt_memset(topic_name, 0, 128);
			
        	if (ef_get_env_blob(mqtt_sub_item[i].topic_filter, topic_name, 127, RT_NULL) <= 0)
				continue;
			
            if (IOT_MQTT_Subscribe(mqtt_device_hd, topic_name, mqtt_sub_item[i].qos, mqtt_sub_item[i].topic_handle_func, mqtt_sub_item[i].pcontext) < 0)
            {
                LOG_D("IOT_MQTT_Subscribe() failed, topic = %s", topic_name);
                IOT_MQTT_Destroy(&mqtt_device_hd);
                goto do_device_exit;
            }
        }

		/* handle the MQTT packet received from TCP or SSL connection */
        IOT_MQTT_Yield(mqtt_device_hd, 200);
		mqtt_is_running = 1;
	
        while (IOT_MQTT_CheckStateNormal(mqtt_device_hd))
        {
            /* handle the MQTT packet received from TCP or SSL connection */
            IOT_MQTT_Yield(mqtt_device_hd, 200);
            rt_thread_delay(rt_tick_from_millisecond(1000));          
        }
		mqtt_is_running = 0;

        /* ubsbuscribe all topic */
        for (i = 0; i < sizeof(mqtt_sub_item) / sizeof(mqtt_subscribe_item); i++)
        {
        	rt_memset(topic_name, 0, 128);
			
        	if (ef_get_env_blob(mqtt_sub_item[i].topic_filter, topic_name, 127, RT_NULL) <= 0)
				continue;
			
            IOT_MQTT_Unsubscribe(mqtt_device_hd, topic_name);
        }

        IOT_MQTT_Destroy(&mqtt_device_hd);

do_device_exit:
        if (RT_NULL != msg_writebuf)
            rt_free(msg_writebuf);

        if (RT_NULL != msg_readbuf)
            rt_free(msg_readbuf);		

		if (RT_NULL != topic_name)
			rt_free(topic_name);

		topic_name = RT_NULL;
        msg_writebuf = RT_NULL;
        msg_readbuf = RT_NULL;       
        
        rt_thread_delay(1000);
    }
}

static int ali_mqtt_init(void)
{
	size_t env_len;
    rt_thread_t tid;

	env_len = HAL_GetProductKey(RT_NULL);
	if (env_len <= 0)
	{
		LOG_D("ProductKey read failed.");
		goto __mqtt_init_exit;	
	}

	env_len = HAL_GetDeviceName(RT_NULL);
	if (env_len <= 0)
	{
		LOG_D("DeviceName read failed.");
		goto __mqtt_init_exit;	
	}

	env_len = HAL_GetDeviceSecret(RT_NULL);
	if (env_len <= 0)
	{
		LOG_D("DeviceSecret read failed.");
		goto __mqtt_init_exit;	
	}	
		
    device_mailbox = rt_mb_create("dev_mbox", 10, RT_IPC_FLAG_FIFO);
    RT_ASSERT(device_mailbox != RT_NULL);

    tid = rt_thread_create("ali-device", mqtt_device, RT_NULL, 6 * 1024, RT_THREAD_PRIORITY_MAX / 2 - 1, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);

__mqtt_init_exit:
    return 0;
}
//INIT_APP_EXPORT(ali_mqtt_init);

static int ali_mqtt_test(int argc, char **argv)
{
    char msg_pub[512];
    iotx_mqtt_topic_info_t topic_msg;

    if (!mqtt_is_running)
    {
        LOG_I("MQTT routine is not running");
        return 0;
    }

    if (argc < 3)
    {
        rt_kprintf("device information need two parameter.");
        return 0;
    }

	if ((!rt_strncmp("devtag", argv[1], 7)) && (argc >= 3))
	{
		/* Initialize topic information */
    	rt_memset(msg_pub, 0, sizeof(msg_pub));
    	snprintf(msg_pub, sizeof(msg_pub), "{\"id\": \"%d\",\"version\": \"1.0\",\"params\": [{\"attrKey\": \"%s\",\"attrValue\": \"%s\"}],\"method\": \"thing.deviceinfo.update\"}",
                                        123, argv[1], argv[2]);
	}

    rt_memset(&topic_msg, 0, sizeof(iotx_mqtt_topic_info_t));
    topic_msg.qos = IOTX_MQTT_QOS1;
    topic_msg.retain = 0;
    topic_msg.dup = 0;
    topic_msg.payload = (void *)msg_pub;
    topic_msg.payload_len = strlen(msg_pub);       

	char topic_name[64];
	rt_memset(topic_name, 0, sizeof(topic_name));
	if (ef_get_env_blob(ALI_DEVICEINFO_UPDATE_PUB, topic_name, sizeof(topic_name), RT_NULL) > 0)
		IOT_MQTT_Publish(mqtt_device_hd, topic_name, &topic_msg);    	
    
    return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(ali_mqtt_init, mqtt, Example: dev_info tbox this is a good device);
MSH_CMD_EXPORT_ALIAS(ali_mqtt_test, dev_info, Example: dev_info tbox this is a good device);
#endif
