/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "cli_model.h"
#include "vendormodel_server.h"
#include "ltrn_extern.h"
#include "cliface.h"
#include "access_internal.h"

#define CONSOLE_OUT(...)    printf(__VA_ARGS__)
#define CONSOLE_IN(...)     scanf(__VA_ARGS__)

extern CLI_COMMAND* g_cli_cmd_list;
extern uint8_t g_cli_cmd_len;

/*********************************************************************
    EXTERNAL VARIABLES
*/
extern EM_timer_handle thandle;
extern uint8 llState;
extern uint8 llSecondaryState;
extern llGlobalStatistics_t g_pmCounters;
extern UCHAR blebrr_state;
extern uint32 blebrr_advscan_timeout_count;
extern UINT32 blebrr_scanTimeOut;

extern llGlobalStatistics_t g_pmCounters;
//extern uint32_t g_stop_scan_t1;
//extern uint32_t g_stop_scan_t1_err;
//extern uint8_t llModeDbg[6];

extern PROV_DEVICE_S UI_lprov_device;


/*********************************************************************
    EXTERNAL FUNCTIONS
*/

#if(BLEMESH_ROLE == PROV_ROLE_PROVISIONER)
#else
extern MS_ACCESS_MODEL_HANDLE   UI_vendor_defined_server_model_handle;
extern API_RESULT UI_sample_get_net_key(void );
extern API_RESULT UI_sample_get_device_key(void);
extern API_RESULT UI_sample_check_app_key(void);

extern void timeout_cb (void* args, UINT16 size);



/*********************************************************************
    LOCAL FUNCTIONS
*/
static API_RESULT cli_vendormodel_send_reliable_pdu(
    /* IN */ UINT32    req_opcode,
    /* IN */ UINT16    dest_addr,
    /* IN */ UINT16    appKey_index,
    /* IN */ void*     param,
    /* IN */ UINT16    len
)
{
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[384];
    UCHAR*     pdu_ptr;
    UINT16     marker;
    retval = API_FAILURE;
    marker = 0;

    switch(req_opcode)
    {
    case MS_ACCESS_VENDORMODEL_WRITECMD_OPCODE:
    {
        EM_mem_copy(&buffer[marker], param, len);
        marker += len;
    }
    break;

    default:
        break;
    }

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
    }

    retval = MS_access_raw_data
             (
                 &UI_vendor_defined_server_model_handle,
                 req_opcode,
                 dest_addr,
                 appKey_index,
                 pdu_ptr,
                 marker,
                 MS_FALSE
             );
    return retval;
}


API_RESULT cli_raw_data(UINT32 argc, UCHAR* argv[])
{
    API_RESULT retval;
    UINT16  destnation_address;
    UINT16   data_len,appKeyIndex;
    UINT8   buffer[256];

    if( argc < 3 )
    {
        printf("Invaild RAW DATA Paraments\n");
        return API_FAILURE;
    }

    destnation_address = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    appKeyIndex = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
    data_len = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);

    if(data_len == 0)
    {
        printf("No RAW DATA,Return\n");
        return API_FAILURE;
    }

    retval = CLI_strtoarray
             (
                 argv[3],
                 CLI_strlen(argv[3]),
                 buffer,
                 data_len
             );

    if(retval != API_SUCCESS)
    {
        return retval;
    }

    cli_vendormodel_send_reliable_pdu(
        MS_ACCESS_VENDORMODEL_WRITECMD_OPCODE,
        destnation_address,
        appKeyIndex,
        buffer,
        data_len
    );
    printf("destnation_address 0x%04X data_len 0x%02X\n",destnation_address,data_len);
    return API_SUCCESS;
}
#endif

API_RESULT cli_get_information(UINT32 argc, UCHAR* argv[])
{
    API_RESULT retval;
    MS_NET_ADDR addr;
    UINT8 features;
    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);
    retval = MS_access_cm_get_primary_unicast_address(&addr);

    if(retval != API_SUCCESS)
    {
        return API_FAILURE;
    }

    MS_access_cm_get_features(&features);
    dbg_printf("[ATMSH81]%04X%02X",addr,features);
    return API_SUCCESS;
}

API_RESULT cli_mesh_stack_clear(UINT32 argc, UCHAR* argv[])
{
    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);
    printf("cli mesh stack clear\n");
    #if(BLEMESH_ROLE == PROV_ROLE_PROVISIONER)
    MS_access_cm_reset(PROV_ROLE_PROVISIONER);
    #else
    UCHAR proxy_state;
    MS_proxy_fetch_state(&proxy_state);
    MS_access_cm_reset(PROV_ROLE_DEVICE);
    blebrr_scan_pl(FALSE);

    if(MS_PROXY_CONNECTED != proxy_state)
    {
        EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
    }
    else
    {
        blebrr_disconnect_pl();
    }

    #endif
    return API_SUCCESS;
}
#if(BLEMESH_ROLE == PROV_ROLE_PROVISIONER)
#else
API_RESULT cli_disp_key(UINT32 argc, UCHAR* argv[])
{
    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);
    UI_sample_get_net_key();
    UI_sample_get_device_key();
    UI_sample_check_app_key();
    return API_SUCCESS;
}

API_RESULT cli_send_pdu(UINT32 argc, UCHAR* argv[])
{
    UINT16  destnation_address;
    UINT16   data_len,appKeyIndex;
    UINT8   buffer[384];

    if( argc < 3 )
    {
        printf("Invaild RAW DATA Paraments\n");
        return API_FAILURE;
    }

    destnation_address = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    appKeyIndex = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
    data_len = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);

    if((data_len == 0) || data_len >384)
    {
        printf("RAW DATA INVALID,Return\n");
        return API_FAILURE;
    }

    for(int i=0; i<data_len; i++)
    {
        buffer[i] = i;
    }

    printf("-before-\n");
    #if (MESH_HEAP == 1)
    extern uint32  mesh_osal_memory_statics(void);
    mesh_osal_memory_statics();
    #else
    extern uint32  osal_memory_statics(void);
    osal_memory_statics();
    #endif
    cli_vendormodel_send_reliable_pdu(
        MS_ACCESS_VENDORMODEL_WRITECMD_OPCODE,
        destnation_address,
        appKeyIndex,
        buffer,
        data_len
    );
    printf("-after-\n");
    #if (MESH_HEAP == 1)
    mesh_osal_memory_statics();
    #else
    osal_memory_statics();
    #endif
    printf("DST 0x%04X data_len 0x%02X\n",destnation_address,data_len);
    return API_SUCCESS;
}

#endif

static void ll_dumpConnectionInfo(void )
{
    printf("========== LL PM counters ================\r\n");
    printf("ll_send_undirect_adv_cnt = %d\r\n", g_pmCounters.ll_send_undirect_adv_cnt);
    printf("ll_send_nonconn_adv_cnt = %d\r\n", g_pmCounters.ll_send_nonconn_adv_cnt);
    printf("ll_send_scan_adv_cnt = %d\r\n", g_pmCounters.ll_send_scan_adv_cnt);
    printf("ll_send_hdc_dir_adv_cnt = %d\r\n", g_pmCounters.ll_send_hdc_dir_adv_cnt);
    printf("ll_send_ldc_dir_adv_cnt = %d\r\n", g_pmCounters.ll_send_ldc_dir_adv_cnt);
    printf("ll_send_conn_adv_cnt = %d\r\n", g_pmCounters.ll_send_conn_adv_cnt);
    printf("ll_conn_adv_pending_cnt = %d\r\n", g_pmCounters.ll_conn_adv_pending_cnt);
    printf("ll_conn_scan_pending_cnt = %d\r\n", g_pmCounters.ll_conn_scan_pending_cnt);
    printf("ll_recv_scan_req_cnt = %d\r\n", g_pmCounters.ll_recv_scan_req_cnt);
    printf("ll_send_scan_rsp_cnt = %d\r\n", g_pmCounters.ll_send_scan_rsp_cnt);
    printf("ll_recv_conn_req_cnt = %d\r\n", g_pmCounters.ll_recv_conn_req_cnt);
    printf("ll_send_conn_rsp_cnt = %d\r\n", g_pmCounters.ll_send_conn_rsp_cnt);
    printf("ll_filter_scan_req_cnt = %d\r\n", g_pmCounters.ll_filter_scan_req_cnt);
    printf("ll_filter_conn_req_cnt = %d\r\n", g_pmCounters.ll_filter_conn_req_cnt);
    printf("ll_recv_adv_pkt_cnt = %d\r\n", g_pmCounters.ll_recv_adv_pkt_cnt);
    printf("ll_send_scan_req_cnt = %d\r\n", g_pmCounters.ll_send_scan_req_cnt);
    printf("ll_recv_scan_rsp_cnt = %d\r\n", g_pmCounters.ll_recv_scan_rsp_cnt);
    printf("ll_conn_succ_cnt = %d\r\n", g_pmCounters.ll_conn_succ_cnt);
    printf("ll_link_lost_cnt = %d\r\n", g_pmCounters.ll_link_lost_cnt);
    printf("ll_link_estab_fail_cnt = %d\r\n", g_pmCounters.ll_link_estab_fail_cnt);
    printf("ll_rx_peer_cnt = %d\r\n", g_pmCounters.ll_rx_peer_cnt);
    printf("ll_evt_shc_err = %d\r\n", g_pmCounters.ll_evt_shc_err);
    printf("ll_trigger_err = %d\r\n", g_pmCounters.ll_trigger_err);
    printf("ll_rfifo_rst_err = %d\r\n", g_pmCounters.ll_rfifo_rst_err);
    printf("ll_rfifo_rst_cnt = %d\r\n", g_pmCounters.ll_rfifo_rst_cnt);
    printf("ll_rfifo_read_err = %d\r\n", g_pmCounters.ll_rfifo_read_err);
    printf("\r\n ");
}

API_RESULT cli_demo_reset(UINT32 argc, UCHAR* argv[])
{
    UCHAR  proxy_state,proxy;
    MS_access_cm_get_features_field(&proxy, MS_FEATURE_PROXY);

    if(MS_TRUE == proxy)
    {
        MS_proxy_fetch_state(&proxy_state);

        if(proxy_state == MS_PROXY_CONNECTED)
            blebrr_disconnect_pl();
    }

    #if(BLEMESH_ROLE == PROV_ROLE_PROVISIONER)
    MS_access_cm_reset(PROV_ROLE_PROVISIONER);
    extern MS_PROV_DEV_ENTRY g_prov_dev_list[MS_MAX_DEV_KEYS];

    for(int i=0; i<MS_MAX_DEV_KEYS; i++)
        EM_mem_set(&g_prov_dev_list[i],0,sizeof(MS_PROV_DEV_ENTRY));

    ms_access_ps_store(MS_PS_RECORD_DEV_KEYS);
    #else
    blebrr_scan_pl(FALSE);
    nvs_reset(NVS_BANK_PERSISTENT);
    MS_access_cm_reset(PROV_ROLE_DEVICE);

    if(thandle == EM_TIMER_HANDLE_INIT_VAL)
    {
        EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
    }

    #endif
    //printf ("Done\r\n");
    CLIT_output("[%s,%d]","reset",API_SUCCESS);
    return API_SUCCESS;
}




API_RESULT cli_internal_status(UINT32 argc, UCHAR* argv[])
{
    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);
    printf("\r\n===== internal status ============\r\n");
    printf("llState = %d, llSecondaryState = %d\r\n", llState, llSecondaryState);
    printf("blebrr_state = %d\r\n", blebrr_state);
    printf("blebrr_scanTimOut = %d\r\n", blebrr_scanTimeOut);
    printf("\r\n");
    ll_dumpConnectionInfo();
    return API_SUCCESS;
}

/* Send Config Heartbeat Publication Set */
API_RESULT cli_modelc_config_heartbeat_publication_set(UINT32 argc, UCHAR* argv[])
{
    API_RESULT retval;
    int  choice;
    ACCESS_CONFIG_HEARTBEATPUB_SET_PARAM  param;
    CONSOLE_OUT
    (">> Send Config Heartbeat Publication Set\n");

    if (6 == argc)
    {
        choice = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
        param.destination = (UINT16)choice;
        CONSOLE_OUT("Destination (16-bit in HEX): 0x%04X\n", param.destination);
        choice = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
        param.countlog = (UCHAR)choice;
        CONSOLE_OUT("CountLog (8-bit in HEX): 0x%02X\n", param.countlog);
        choice = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
        param.periodlog = (UCHAR)choice;
        CONSOLE_OUT("PeriodLog (8-bit in HEX): 0x%02X\n", param.periodlog);
        choice = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 16);
        param.ttl = (UCHAR)choice;
        CONSOLE_OUT("TTL (8-bit in HEX): 0x%02X\n", param.ttl);
        choice = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 16);
        param.features = (UINT16)choice;
        CONSOLE_OUT("Features (16-bit in HEX): 0x%04X\n", param.features);
        choice = CLI_strtoi(argv[5], CLI_strlen(argv[5]), 16);
        param.netkey_index = (UINT16)choice;
        CONSOLE_OUT("NetKeyIndex (16-bit in HEX): 0x%04X\n", param.netkey_index);
    }
    else
    {
        CONSOLE_OUT("Invalid Number of Arguments:0x%04X. Returning.\n", argc);
        return API_FAILURE;
    }

    retval = MS_config_client_heartbeat_publication_set(&param);
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
    return retval;
}

API_RESULT cli_on(UINT32 argc, UCHAR* argv[])
{
//    UI_generic_onoff_set(0x01);
    return API_SUCCESS;
}

API_RESULT cli_off(UINT32 argc, UCHAR* argv[])
{
//    UI_generic_onoff_set(0x00);
    return API_SUCCESS;
}


static void set_uuid_octet (UCHAR uuid_0)
{
}


API_RESULT cli_start(UINT32 argc, UCHAR* argv[])
{
    int val;

    if (1 != argc)
    {
        printf("Usage: start <octet_0>\r\n");
        return API_FAILURE;
    }

    val = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    set_uuid_octet ((UCHAR)val);
//    appl_mesh_sample();
    return API_SUCCESS;
}

API_RESULT cli_group_select(UINT32 argc, UCHAR* argv[])
{
    UINT16  group;

    if (1 != argc)
    {
        printf("Usage: group <Group Idx>\r\n");
        return API_FAILURE;
    }

    group  = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    printf("select Mesh Group %d, behavior to be implemented\r\n", group);
    return API_SUCCESS;
}

/* Send Config Key Refresh Phase Set */
API_RESULT cli_core_modelc_config_key_refresh_phase_set(UINT32 argc, UCHAR* argv[])
{
    API_RESULT retval;
    int  choice;
    ACCESS_CONFIG_KEYREFRESH_PHASE_SET_PARAM  param;
    printf
    (">> Send Config Key Refresh Phase Set\n");

    if (2 == argc)
    {
        choice = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
        param.netkey_index = (UINT16)choice;
        printf("NetKeyIndex (16-bit in HEX): 0x%04X\n", param.netkey_index);
        choice = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
        param.transition = (UCHAR)choice;
        printf("Transition (8-bit in HEX): 0x%02X\n", param.transition);
    }
    else
    {
        printf("Invalid Number of Arguments:0x%04X. Returning.\n", argc);
        return API_FAILURE;
    }

    /* Change Local State as well */
    MS_access_cm_set_key_refresh_phase
    (
        0, /* subnet_handle */
        &param.transition /* key_refresh_state */
    );
    param.transition = (UCHAR)choice;
    retval = MS_config_client_keyrefresh_phase_set(&param);
    printf
    ("retval = 0x%04X\n", retval);
    return retval;
}

/* Send Config Netkey Update */
API_RESULT cli_core_modelc_config_netkey_update(UINT32 argc, UCHAR* argv[])
{
    API_RESULT retval;
    int  choice;
    ACCESS_CONFIG_NETKEY_UPDATE_PARAM  param;
    printf(">> Send Config Netkey Update\n");

    if (2 == argc)
    {
        choice = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
        param.netkey_index = (UINT16)choice;
        printf("NetKeyIndex (16-bit in HEX): 0x%04X\n", param.netkey_index);
        CLI_strtoarray
        (
            argv[1],
            CLI_strlen(argv[1]),
            &param.netkey[0],
            16
        );
    }
    else
    {
        printf("Invalid Number of Arguments:0x%04X. Returning.\n", argc);
        return API_FAILURE;
    }

    /* Set Local NetKey */
    MS_access_cm_add_update_netkey
    (
        0, /* netkey_index */
        MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE, /* opcode */
        &param.netkey[0] /* net_key */
    );
    retval = MS_config_client_netkey_update(&param);
    printf("retval = 0x%04X\n", retval);
    return retval;
}


API_RESULT cli_snb(UINT32 argc, UCHAR* argv[])
{
    extern NET_SEQ_NUMBER_STATE net_seq_number_state;
    printf ("Set sequnce number\r\n");
    UINT8 auto_flag, ivindex, ivflag;
    auto_flag = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    ivindex = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 10);
    ivflag = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);

    if(auto_flag == 0)
    {
        net_seq_number_state.seq_num = 0xEFFFF0;
        net_seq_number_state.block_seq_num_max = 0xEFFFF0;
    }
    else if(auto_flag == 1)
    {
        ms_iv_index.iv_index = ivindex;

        if(ivflag == 0x00)
        {
            ms_iv_index.iv_update_state &= ivflag;
        }
        else
        {
            ms_iv_index.iv_update_state |= ivflag;
        }
    }
    else if(auto_flag == 2)
    {
        if(ivindex == 0)
        {
            MS_DISABLE_SNB_FEATURE();
            MS_net_stop_snb_timer(0);
        }
        else
        {
            MS_ENABLE_SNB_FEATURE();
            MS_net_start_snb_timer(0);
        }
    }

    MS_access_ps_store_all_record();
    printf ("Done\r\n");
    return API_SUCCESS;
}

MS_NET_HEADER    g_hdr;
UCHAR            g_hdr_flag = MS_FALSE;
API_RESULT cli_seg_send(UINT32 argc, UCHAR* argv[])
{
    UINT16           dst_addr;
    UCHAR            mode;
    API_RESULT       retval;
    UINT16           len;
    UCHAR*             trans_pdu;
    /* Un-Segmented Data of 5 Bytes */
    DECL_CONST UCHAR unseg_trans_pdu[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
    /* Segmented Data of 15 Bytes */
    DECL_CONST UCHAR seg_trans_pdu[]   = { 0x01, 0x02, 0x03, 0x04, 0x05,
                                           0x06, 0x07, 0x08, 0x09, 0x0A,
                                           0x0B, 0x0C, 0x0D, 0x0E, 0x0F
                                         };

    if (2 != argc)
    {
        printf("Usage: send <destination addr> <Flag[0: Unsegmented, 1: Segmented]>\n");
        return API_FAILURE;
    }

    dst_addr = (UINT16)CLI_strtoi(argv[0], (UINT8)CLI_strlen(argv[0]), 16);
    mode     = (UCHAR) CLI_strtoi(argv[1], (UINT8)CLI_strlen(argv[1]), 16);

    if (MS_FALSE == g_hdr_flag)
    {
        /* Initialize */
        EM_mem_set(&g_hdr, 0x0, sizeof(g_hdr));
        MS_access_cm_get_primary_unicast_address(&g_hdr.saddr);
        g_hdr.daddr = dst_addr;
        g_hdr.ttl   = 0x01;
        g_hdr.ctl   = 0x00;
    }

    /* Update Sequence Number irrespective of g_hdr_flag */
    MS_net_alloc_seq_num(&g_hdr.seq_num);

    /* Set PDU */
    if (0 == mode)
    {
        trans_pdu = (UCHAR*)unseg_trans_pdu;
        len       = sizeof(unseg_trans_pdu);
    }
    else
    {
        trans_pdu = (UCHAR*)seg_trans_pdu;
        len       = sizeof(seg_trans_pdu);
    }

    retval = MS_access_send_pdu(g_hdr.saddr, dst_addr, 0x0000, 0x0000, 2,0x00E20405,trans_pdu, len, 0);
    printf("retval:%x",retval)
    return retval;
}


/*********************************************************************
    PUBLIC FUNCTIONS
*/

API_RESULT cli_demo_help(UINT32 argc, UCHAR* argv[])
{
    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);
    CLI_help(g_cli_cmd_list, g_cli_cmd_len);
    return API_SUCCESS;
}


/*********************************************************************
*********************************************************************/
