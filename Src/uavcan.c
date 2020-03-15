#include <stdio.h>
#include <stdlib.h>

#include "uavcan.h"

#include "main.h"

#define CANARD_SPIN_PERIOD   100
#define PUBLISHER_PERIOD_mS     50

static CanardInstance g_canard;                //The library instance
static uint8_t g_canard_memory_pool[1024];     //Arena for memory allocation, used by the library
static uint32_t  g_uptime = 0;
uint16_t rc_pwm[6] = {0,0,0,0,0,0};
//////////////////////////////////////////////////////////////////////////////////////
bool shouldAcceptTransfer(const CanardInstance* ins,
                          uint64_t* out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id)
{
    if ((transfer_type == CanardTransferTypeRequest) &&(data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
        *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
        return true;
    }
    if (data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)
    {
        *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
        return true;
    }
    if (data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID)
    {
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////

void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if ((transfer->transfer_type == CanardTransferTypeRequest) && (transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
        getNodeInfoHandleCanard(transfer);
    }

    if (transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)
    {
        rawcmdHandleCanard(transfer);
    }

    if (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID)
    {
        getsetHandleCanard(transfer);
    }

}

void getNodeInfoHandleCanard(CanardRxTransfer* transfer)
{
        uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
        memset(buffer,0,UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
        uint16_t len = makeNodeInfoMessage(buffer);
        int result = canardRequestOrRespond(&g_canard,
                                            transfer->source_node_id,
                                            UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
                                            UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
                                            &transfer->transfer_id,
                                            transfer->priority,
                                            CanardResponse,
                                            &buffer[0],
                                            (uint16_t)len);
}

void uavcanInit(void)
{
    CanardSTM32CANTimings timings;
    int result = canardSTM32ComputeCANTimings(HAL_RCC_GetPCLK1Freq(), 1000000, &timings);
    if (result)
    {
        __ASM volatile("BKPT #01");
    }
    result = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
    if (result)
    {
        __ASM volatile("BKPT #01");
    }

    canardInit(&g_canard,                         // Uninitialized library instance
               g_canard_memory_pool,              // Raw memory chunk used for dynamic allocation
               sizeof(g_canard_memory_pool),      // Size of the above, in bytes
               onTransferReceived,                // Callback, see CanardOnTransferReception
               shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
               NULL);

    canardSetLocalNodeID(&g_canard, 11);
}

void sendCanard(void)
{
  const CanardCANFrame* txf = canardPeekTxQueue(&g_canard);
  while(txf)
    {
        const int tx_res = canardSTM32Transmit(txf);
        if (tx_res < 0)                  // Failure - drop the frame and report
        {
            __ASM volatile("BKPT #01");  // TODO: handle the error properly
        }
        if(tx_res > 0)
        {
            canardPopTxQueue(&g_canard);
        }
        txf = canardPeekTxQueue(&g_canard);
    }
}

void receiveCanard(void)
{
    CanardCANFrame rx_frame;
    int res = canardSTM32Receive(&rx_frame);
    if(res)
    {
        canardHandleRxFrame(&g_canard, &rx_frame, HAL_GetTick() * 1000);
    }
}

void spinCanard(void)
{
    static uint32_t spin_time = 0;
    if(HAL_GetTick() < spin_time + CANARD_SPIN_PERIOD) return;  // rate limiting
    spin_time = HAL_GetTick();
    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
    static uint8_t transfer_id = 0;                           // This variable MUST BE STATIC; refer to the libcanard documentation for the background
    makeNodeStatusMessage(buffer);
    const int16_t bc_res = canardBroadcast(&g_canard,
                    UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                    UAVCAN_NODE_STATUS_DATA_TYPE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    UAVCAN_NODE_STATUS_MESSAGE_SIZE);                         //some indication

    if(bc_res <=0){
    	//handle errr
    }


}

void publishCanard(void)
{
    static uint32_t publish_time = 0;
    static int step = 0;
    if(HAL_GetTick() < publish_time + PUBLISHER_PERIOD_mS) {return;} // rate limiting
    publish_time = HAL_GetTick();
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);


    /*
    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MESSAGE_SIZE];
    memset(buffer,0x00,UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MESSAGE_SIZE);
    step++;
    if(step == 2560)
    {
        step = 0;
    }
    //float val = step;



    static uint8_t transfer_id = 0;
    canardEncodeScalar(buffer, 0, 32, &val);
    memcpy(&buffer[4], "sin", 3);
    canardBroadcast(&g_canard,
                    UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE,
                    UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    7);
    memset(buffer,0x00,UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MESSAGE_SIZE);

    val = step;
    canardEncodeScalar(buffer, 0, 32, &val);
    memcpy(&buffer[4], "stp", 3);
    canardBroadcast(&g_canard,
                    UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE,
                    UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    7);
	*/
    //publishAirspeed();

}

void makeIndicatedAirspeedMessage(uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_INDICATED_MESSAGE_SIZE])
{
	/*
	 * IndicatedAirspeed
	 * Full name: uavcan.equipment.air_data.IndicatedAirspeed
     * Default data type ID: 1021
     * float16 indicated_airspeed              # m/s
     * float16 indicated_airspeed_variance     # (m/s)^2
     * signature: 0x0a1892d72ab8945f
     * size: 16b+16b=32b, 32b/8 = 4B (2 Bytes)
	 */
	memset(buffer, 0, UAVCAN_EQUIPMENT_AIR_DATA_INDICATED_MESSAGE_SIZE);

	/*
	 * uint16_t canardConvertNativeFloatToFloat16(float value);
	 * float canardConvertFloat16ToNativeFloat(uint16_t value);
	 */


	 float indicated_airspeed_native = 123.456;
	 float indicated_airspeed_variance_native = 2.5;
	 uint16_t indicated_airspeed = canardConvertNativeFloatToFloat16(indicated_airspeed_native);
	 uint16_t indicated_airspeed_variance = canardConvertNativeFloatToFloat16(indicated_airspeed_variance_native);
	 canardEncodeScalar(buffer, 0, 16, &indicated_airspeed);
	 canardEncodeScalar(buffer, 16, 16, &indicated_airspeed_variance);
}

uint16_t convertFrom16To32(uint16_t dataFirst, uint16_t dataSecond) {
    uint32_t dataBoth = 0x00000000;

    dataBoth = dataFirst;
    dataBoth = dataBoth << 16;
    dataBoth |= dataSecond;
    return dataBoth;
}




void makeRawAirDatadMessage(uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIZE], float diff_press, float temp)
{
	/*
	 * Full name: uavcan.equipment.air_data.RawAirData
     * Default data type ID: 1027
	 */

	memset(buffer, 0, UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIZE);

	/*
	 * uint16_t canardConvertNativeFloatToFloat16(float value);
	 * float canardConvertFloat16ToNativeFloat(uint16_t value);
	 */

	 uint8_t flags = 1;
	 uint16_t pad= 0;
	 //uint32_t static_pressure = convertFrom16To32(canardConvertNativeFloatToFloat16(987.456), 0);
	 //uint32_t differential_pressure = convertFrom16To32(0, canardConvertNativeFloatToFloat16(1000));
	 //uint16_t static_pressure = canardConvertNativeFloatToFloat16(4.1);
	 //uint16_t differential_pressure = canardConvertNativeFloatToFloat16(0.0);
	 float static_pressure = 101300.0;
	 //float differential_pressure = 1000.0;
	 float differential_pressure = diff_press;


	 uint16_t static_pressure_sensor_temperature = 0.0;
	 uint16_t differential_pressure_sensor_temperature = 0;
	 uint16_t static_air_temperature = canardConvertNativeFloatToFloat16(temp+273.15);
	 uint16_t pitot_temperature = 10;
	 //uint16_t padding[16];


	 canardEncodeScalar(buffer, 0, 8, &flags);
	 canardEncodeScalar(buffer, 8, 32, &static_pressure);
	 canardEncodeScalar(buffer, 40, 32, &differential_pressure);
	 canardEncodeScalar(buffer, 72, 16, &static_pressure_sensor_temperature);
	 canardEncodeScalar(buffer, 88, 16, &differential_pressure_sensor_temperature);
	 canardEncodeScalar(buffer, 104, 16, &static_air_temperature);
	 canardEncodeScalar(buffer, 120, 16, &pitot_temperature);
	 //canardEncodeScalar(buffer, 136, 16, &pitot_temperature);


}

void publishAirspeed(float diff_press, float temp)
{
/*
	uint8_t buff[UAVCAN_EQUIPMENT_AIR_DATA_INDICATED_MESSAGE_SIZE];
	makeIndicatedAirspeedMessage(buff);
	static uint8_t transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!
	const int16_t bc_res = canardBroadcast(&g_canard, UAVCAN_EQUIPMENT_AIR_DATA_INDICATED_AIRSPEED_SIGNATURE, UAVCAN_EQUIPMENT_AIR_DATA_INDICATED_AIRSPEED_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_HIGHEST, buff, UAVCAN_EQUIPMENT_AIR_DATA_INDICATED_MESSAGE_SIZE);
	if(bc_res <= 0)
	{
		//handle error
	}

*/

	/*
	static uint32_t publish_time = 0;
	static int step = 0;
	if(HAL_GetTick() < publish_time + PUBLISHER_PERIOD_mS) {return;} // rate limiting
	publish_time = HAL_GetTick();
	*/

	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	uint8_t buff[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIZE];
	makeRawAirDatadMessage(buff, diff_press, temp);
	static uint8_t transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!
	const int16_t airspeed_result = canardBroadcast(&g_canard, UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE, UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_HIGHEST, buff, UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIZE);
	if(airspeed_result <= 0)
	{
		//handle error
	}


}




void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
    uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
    uint8_t node_mode   = UAVCAN_NODE_MODE_OPERATIONAL;
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
    uint32_t uptime_sec = (HAL_GetTick() / 1000);
    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}

uint16_t makeNodeInfoMessage(uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE])
{
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    makeNodeStatusMessage(buffer);

    buffer[7] = APP_VERSION_MAJOR;
    buffer[8] = APP_VERSION_MINOR;
    buffer[9] = 1;                          // Optional field flags, VCS commit is set
    uint32_t u32 = GIT_HASH;
    canardEncodeScalar(buffer, 80, 32, &u32);

    readUniqueID(&buffer[24]);
    const size_t name_len = strlen(APP_NODE_NAME);
    memcpy(&buffer[41], APP_NODE_NAME, name_len);
    return 41 + name_len ;
}

void readUniqueID(uint8_t* out_uid)
{
    for (uint8_t i = 0; i < UNIQUE_ID_LENGTH_BYTES; i++)
    {
        out_uid[i] = i;
    }
}


void rawcmdHandleCanard(CanardRxTransfer* transfer)
{

    int offset = 0;
    for (int i = 0; i<6; i++)
    {
        if (canardDecodeScalar(transfer, offset, 14, true, &rc_pwm[i])<14) { break; }
        offset += 14;
    }
   // rcpwmUpdate(ar);
}

void showRcpwmonUart()
{
    char str[5];
    itoa(rc_pwm[0],str,10);
    //HAL_UART_Transmit(&huart1,str,5,0xffff);
    //HAL_UART_Transmit(&huart1,"\n",2,0xffff);
    //CDC_Transmit_FS(str, 5);
    //CDC_Transmit_FS("\n", 2);
}


param_t parameters[] =
{
    {"param0", 0, 10,20, 15},
    {"param1", 1, 0, 100, 25},
    {"param2", 2, 2, 8,  3 },
};

param_t * getParamByIndex(uint16_t index)
{
  if(index >= ARRAY_SIZE(parameters))
  {
    return NULL;
  }

  return &parameters[index];
}

param_t * getParamByName(uint8_t * name)
{
  for(uint16_t i = 0; i < ARRAY_SIZE(parameters); i++)
  {
    if(strncmp((char const*)name, (char const*)parameters[i].name,strlen((char const*)parameters[i].name)) == 0)
    {
      return &parameters[i];
    }
  }
  return NULL;
}

uint16_t encodeParamCanard(param_t * p, uint8_t * buffer)
{
    uint8_t n     = 0;
    int offset    = 0;
    uint8_t tag   = 1;
    if(p==NULL)
    {
        tag = 0;
        canardEncodeScalar(buffer, offset, 5, &n);
        offset += 5;
        canardEncodeScalar(buffer, offset,3, &tag);
        offset += 3;

        canardEncodeScalar(buffer, offset, 6, &n);
        offset += 6;
        canardEncodeScalar(buffer, offset,2, &tag);
        offset += 2;

        canardEncodeScalar(buffer, offset, 6, &n);
        offset += 6;
        canardEncodeScalar(buffer, offset, 2, &tag);
        offset += 2;
        buffer[offset / 8] = 0;
        return ( offset / 8 + 1 );
    }
    canardEncodeScalar(buffer, offset, 5,&n);
    offset += 5;
    canardEncodeScalar(buffer, offset, 3, &tag);
    offset += 3;
    canardEncodeScalar(buffer, offset, 64, &p->val);
    offset += 64;

    canardEncodeScalar(buffer, offset, 5, &n);
    offset += 5;
    canardEncodeScalar(buffer, offset, 3, &tag);
    offset += 3;
    canardEncodeScalar(buffer, offset, 64, &p->defval);
    offset += 64;

    canardEncodeScalar(buffer, offset, 6, &n);
    offset += 6;
    canardEncodeScalar(buffer, offset, 2, &tag);
    offset += 2;
    canardEncodeScalar(buffer, offset, 64, &p->max);
    offset += 64;

    canardEncodeScalar(buffer, offset, 6, &n);
    offset += 6;
    canardEncodeScalar(buffer, offset,2,&tag);
    offset += 2;
    canardEncodeScalar(buffer, offset,64,&p->min);
    offset += 64;

    memcpy(&buffer[offset / 8], p->name, strlen((char const*)p->name));
    return  (offset/8 + strlen((char const*)p->name));
}


void getsetHandleCanard(CanardRxTransfer* transfer)
{
    uint16_t index = 0xFFFF;
    uint8_t tag    = 0;
    int offset     = 0;
    int64_t val    = 0;

    canardDecodeScalar(transfer, offset,  13, false, &index);
    offset += 13;
    canardDecodeScalar(transfer, offset, 3, false, &tag);
    offset += 3;

    if(tag == 1)
    {
        canardDecodeScalar(transfer, offset, 64, false, &val);
        offset += 64;
    }

    uint16_t n = transfer->payload_len - offset / 8 ;
    uint8_t name[16]      = "";
    for(int i = 0; i < n; i++)
    {
        canardDecodeScalar(transfer, offset, 8, false, &name[i]);
        offset += 8;
    }

    param_t * p = NULL;

    if(strlen((char const*)name))
    {
        p = getParamByName(name);
    }
    else
    {
        p = getParamByIndex(index);
    }

    if((p)&&(tag == 1))
    {
        p->val = val;
    }

    uint8_t  buffer[64] = "";
    uint16_t len = encodeParamCanard(p, buffer);
    int result = canardRequestOrRespond(&g_canard,
                                        transfer->source_node_id,
                                        UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                                        UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                                        &transfer->transfer_id,
                                        transfer->priority,
                                        CanardResponse,
                                        &buffer[0],
                                        (uint16_t)len);

}

