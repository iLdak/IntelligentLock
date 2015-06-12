#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "npi.h"
#include "osal_snv.h"
#include "simpleBLE.h"
#include "stdio.h"

SYS_CONFIG sys_config;
static bool g_bToConnect = FALSE;

extern gaprole_States_t gapProfileState;   // 从机连接状态


// AT 命令处理
static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events );
static void simpleBLE_UartDataMain(uint8 *buf, uint8 numBytes);
static bool simpleBLE_AT_CMD_Handle(uint8 *pBuffer, uint16 length);

#if defined (AUTO_UART2UART)  
static void simpleBLE_SendMyData_ForTest();
#endif

void Serial_Delay(int times)
{
  while(times--)
  {
      int i=0;
      for(i=6000;i>0;i--)
      {
    	  asm("nop");
      }
  }
}

static uint8 str_cmp(uint8 *p1,uint8 *p2,uint8 len)
{
  uint8 i=0;
  while(i<len){
    if(p1[i]!=p2[i])
      return 0;
    i++;
  }
  return 1;
}

uint32 str2Num(uint8* numStr, uint8 iLength)
{
    uint8 i = 0;
    int32 rtnInt = 0;
 
    /* 
          为代码简单，在确定输入的字符串都是数字的
          情况下，此处未做检查，否则要检查
          numStr[i] - '0'是否在[0, 9]这个区间内
    */
    for(; i < iLength && numStr[i] != '\0'; ++i)
        rtnInt = rtnInt * 10 + (numStr[i] - '0');    
 
    return rtnInt;
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
#define B_ADDR_STR_LEN                        15

  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

void simpleBLE_SaveAllDataToFlash()
{
    osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);    // 写所有参数
}

//flag: PARA_ALL_FACTORY:  全部恢复出厂设置
//flag: PARA_PARI_FACTORY: 清除配对信息
void simpleBLE_SetAllParaDefault(PARA_SET_FACTORY flag)    
{
    if(flag == PARA_ALL_FACTORY)
    {
        sys_config.baudrate = HAL_UART_BR_9600;  
        //sys_config.baudrate = HAL_UART_BR_115200;  //串口波特率
        sys_config.parity = 0;                                    //流控
        sys_config.stopbit = 0;                                   //停止位

        sys_config.mode = BLE_MODE_SERIAL;         //工作模式 0:透传 ， 1: 直驱 , 2: iBeacon

        sprintf((char*)sys_config.name, "C2HBLE");     //设备名称(设备名称12位)

        sys_config.role = BLE_ROLE_PERIPHERAL;         //主从模式, 默认从机
        //sys_config.role = BLE_ROLE_CENTRAL;

        sprintf((char*)sys_config.pass, "7654321");      //配对密码(7位)
        sys_config.type = 0;                     //鉴权模式，及要不要进行密码配对
        //sys_config.mac_addr[16];               //本机mac地址
        sys_config.connl_status = 0;             //连接最后一次的状态
        sys_config.connect_mac_status = 0;       //连接指定地址的返回状态
        //sys_config.ever_connect_mac_status[MAX_PERIPHERAL_MAC_ADDR][13];       //曾经成功连接过的从机地址
        
        osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*13);
        sprintf((char*)sys_config.verion, "%s", VERSION);       //版本信息 v1.0 ~ v9.9

        sys_config.try_connect_time_ms = 0;       // 

        sys_config.rssi = 0;    //  RSSI 信号值

        sys_config.rxGain = HCI_EXT_RX_GAIN_STD;       //  接收增益强度
        sys_config.txPower = 3;       //  发射信号强度

        sys_config.ibeacon_adver_time_ms = 500;

        sys_config.workMode = 0;      //  模块工作类型  0: 立即工作， 1: 等待AT+CON 或 AT+CONNL 命令
    }
    else if(flag == PARA_PARI_FACTORY)
    {
        osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*13);
        sprintf((char*)sys_config.verion, "%s", VERSION);       //版本信息 v1.0 ~ v9.9
    }
}


void PrintAllPara(void)    
{
#define SERIAL_DELAY     Serial_Delay(10)

    char strTemp[32];
    uint8 i;
    
    sprintf(strTemp, "sys_config.baudrate = %d\r\n", sys_config.baudrate);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.parity = %d\r\n", sys_config.parity);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.stopbit = %d\r\n", sys_config.stopbit);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.mode = %d\r\n", sys_config.mode);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.name = %s\r\n", sys_config.name);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;
    
    sprintf(strTemp, "sys_config.role = %d\r\n", sys_config.role);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;
    
    sprintf(strTemp, "sys_config.pass = %s\r\n", sys_config.pass);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;
    
    sprintf(strTemp, "sys_config.type = %d\r\n", sys_config.type);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.mac_addr = %s\r\n", sys_config.mac_addr);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.connl_status = %d\r\n", sys_config.connl_status);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.connect_mac_status = %d\r\n", sys_config.connect_mac_status);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;
    
    //sys_config.ever_connect_mac_status[MAX_PERIPHERAL_MAC_ADDR][13];       //曾经成功连接过的从机地址
    for(i = 0; i<MAX_PERIPHERAL_MAC_ADDR && sys_config.ever_connect_mac_status[i][0] != 0; i++)
    {
        sprintf(strTemp, "sys_config.ever_connect_mac_status[%d] = %s\r\n", i, sys_config.ever_connect_mac_status[i]);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        SERIAL_DELAY;
    }
    
    sprintf(strTemp, "sys_config.verion = %s\r\n", sys_config.verion);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.try_connect_time_ms = %d\r\n", sys_config.try_connect_time_ms);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.rssi = %d\r\n", sys_config.rssi);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.txPower = %d\r\n", sys_config.txPower);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.ibeacon_adver_time_ms = %d\r\n", sys_config.ibeacon_adver_time_ms);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;

    sprintf(strTemp, "sys_config.workMode = %d\r\n", sys_config.workMode);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    SERIAL_DELAY;
}



BLE_ROLE GetBleRole()
{
    return sys_config.role;
}

// 是否已连接上
bool simpleBLE_IfConnected()
{
    if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
    {          
        //LCD_WRITE_STRING_VALUE( "simpleBLECharHd6=", simpleBLECharHd6, 10, HAL_LCD_LINE_1 );
        //return ( ( simpleBLEState == BLE_STATE_CONNECTED  ) && ( simpleBLECharHd6 != 0 ) );
        return ( simpleBLEState == BLE_STATE_CONNECTED  );
    }
    else
    {
        return (gapProfileState == GAPROLE_CONNECTED);
    }
}

// 增加从机地址
void simpleBLE_SetPeripheralMacAddr(uint8 *pAddr)
{
    if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
    {
       osal_memcpy(sys_config.ever_connect_mac_status[0], pAddr, MAC_ADDR_CHAR_LEN);
    }
}
// 读取从机地址
bool simpleBLE_GetPeripheralMacAddr(uint8 *pAddr)
{
    if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
    {          
       if(sys_config.ever_connect_mac_status[0][0] != 0)
       {
            osal_memcpy(pAddr, sys_config.ever_connect_mac_status[0], MAC_ADDR_CHAR_LEN);
            return TRUE;
       }
    }
    return FALSE;
}


void CheckKeyForSetAllParaDefault(void) //按键按下3秒， 回复出厂设置
{
#if 1    
    return;
#else
    uint8 i;
    uint32 old_time  = 75; 

    while(--old_time)
    {
        if(P0_1 == 1)
        {
            LedSetState(HAL_LED_MODE_ON);  
            Serial_Delay(10);
        }
        else
        {
            LedSetState(HAL_LED_MODE_OFF);  
            return;
        }        
    }

    if(old_time == 0)
    {
        simpleBLE_SetAllParaDefault(PARA_ALL_FACTORY);
        for(i = 0; i < 6; i++)    
        {
            LedSetState(HAL_LED_MODE_ON);  
            Serial_Delay(30);
            LedSetState(HAL_LED_MODE_OFF);
            Serial_Delay(30);
        }            
        HAL_SYSTEM_RESET();     
    }
#endif    
}

void simpleBLE_NPI_init(void)
{
#if 1    
    NPI_InitTransportEx(simpleBLE_NpiSerialCallback, sys_config.baudrate, 
        sys_config.parity, sys_config.stopbit );
#else
    NPI_InitTransport(simpleBLE_NpiSerialCallback);
#endif

    // 开机打印主机还是从机
    if(GetBleRole() == BLE_ROLE_CENTRAL)
    {
        NPI_WriteTransport("Hello World Central\r\n",21);
    }
    else
    {
        NPI_WriteTransport("Hello World Peripheral\r\n",24);
    }
}


void UpdateRxGain(void)
{
    // HCI_EXT_SetRxGainCmd()是用来设置发射功率的. 
    // rxGain - HCI_EXT_RX_GAIN_STD, HCI_EXT_RX_GAIN_HIGH
    HCI_EXT_SetRxGainCmd( HCI_EXT_RX_GAIN_STD );
}

void UpdateTxPower(void)
{
        /*
#define LL_EXT_TX_POWER_MINUS_23_DBM                   0
#define LL_EXT_TX_POWER_MINUS_6_DBM                    1
#define LL_EXT_TX_POWER_0_DBM                          2
#define LL_EXT_TX_POWER_4_DBM                          3
        */
    // HCI_EXT_SetTxPowerCmd()是用来设置发射功率的. 有-23dbm, -6dbm, 0 dbm, +4dbm四个级别. 
    HCI_EXT_SetTxPowerCmd(sys_config.txPower);
}


void LedSetState(uint8 onoff)
{
  HalLedSet( HAL_LED_1, onoff);  //led常亮
}

void simpleBle_SetRssi(int8 rssi)
{
    sys_config.rssi = rssi;
}

void simpleBle_PrintPassword()
{
    char strTemp[24] = {0};
    
    sprintf(strTemp, "Password:%s\r\n", sys_config.pass);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

    Serial_Delay(10);
}

uint8* GetAttDeviceName()
{
    return sys_config.name;
}

bool IFfHavePeripheralMacAddr( void )//主机是否记录了从机地址
{
    if(sys_config.ever_connect_mac_status[0][0] == 0)
    {
        return TRUE;
    }
    else
    {
        return TRUE;
    }
}

bool linkonoff = 0;
void performPeriodicTask( void )
{
    static uint8 count = 0;

    if(CheckIfUse_iBeacon())  
    {
        static attHandleValueNoti_t pReport;
        pReport.len = 2;
        pReport.handle = 0x2E;
        osal_memcpy(pReport.value, "ab", 2);
        GATT_Notification( 0, &pReport, FALSE );

        LedSetState(HAL_LED_MODE_TOGGLE);           //取反
        return;
    }
    
    /*
    连线前，
        主机未记录从机地址时，每秒亮100ms；
        主机记录从机地址时，每秒亮900ms；
        从机每2秒亮1秒。
    连线后，
        主机与从机均为，LED每5秒亮100毫秒。
    */
    if(!simpleBLE_IfConnected())
    {    
         if(linkonoff == 1)
        {
		NPI_WriteTransport("AT+UNLINK\r\n",11);
		linkonoff = 0;
	 }
		 
        if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
        {     
            if(IFfHavePeripheralMacAddr() == FALSE)//未记录地址
            {
                if(count == 0)
                {
                    LedSetState(HAL_LED_MODE_ON);  
                } 
                else if(count == 1)
                {
                    LedSetState(HAL_LED_MODE_OFF);
                }
            }
            else
            {
                if(count == 0)
                {
                    LedSetState(HAL_LED_MODE_ON);  
                } 
                else if(count == 9)
                {
                    LedSetState(HAL_LED_MODE_OFF);
                }
            }                    
            count++;
            count %= 10;     
        }  
        else//从机
        {
            if(count == 0)
            {
                LedSetState(HAL_LED_MODE_OFF);  
            } 
            else if(count == 10)
            {
                LedSetState(HAL_LED_MODE_ON);
            }

            count++;
            count %= 20;
        }
    }
    else// 连接后 主机与从机均为，LED每5秒亮100毫秒。(如果想省电， 可以不点灯)
    {
        if(linkonoff == 0)
        {
		NPI_WriteTransport("AT+LINK\r\n",9);
		linkonoff = 1;
	 }
		
        if(count == 0)
        {
            LedSetState(HAL_LED_MODE_ON);  
        } 
        else if(count == 1)
        {
            LedSetState(HAL_LED_MODE_OFF);
        }
        count++;
        count %= 50; 

#if defined (AUTO_UART2UART)  
        // 发送自己的自定义数据， 实现自动数据串口透传
        simpleBLE_SendMyData_ForTest();
#endif

    }
}

bool simpleBle_GetIfNeedPassword()
{
    /*
    Para: 0 ~ 1 
    0: 连接不需要密码
    1: 连接需要密码
    */
    
    return sys_config.type;
}


bool CheckIfUse_iBeacon()
{
    return (sys_config.mode == BLE_MODE_iBeacon);
}

void simpleBLE_SetToConnectFlag(bool bToConnect)
{
    g_bToConnect = bToConnect;
}

bool simpleBLE_GetToConnectFlag(uint8 *Addr)
{
    osal_memcpy(Addr, sys_config.connect_mac_addr, MAC_ADDR_CHAR_LEN);
    /*
    0: 立即工作， 
    1: 等待AT+CON 或 AT+CONNL 命令
    */
    if(sys_config.workMode == 0)
    {
        g_bToConnect = TRUE;
    }
    
    return g_bToConnect;
}

uint32 Get_iBeaconAdvertisingInterral()
{
    return sys_config.ibeacon_adver_time_ms;    
}

#if 1
// 串口回调函数， 下面把该回调函数里实现的功能讲解一下
/*
1, 思路:  当串口收到数据后，就会马上调用以下回调函数，在实际测试中发现，此回调
函数调用频繁， 如果你不执行NPI_ReadTransport函数进行读取， 那么这个回调函数就会
频繁地被执行，但是，你通过串口发送一段数据， 你本意是想处理这一完整一段的数据，所以，
我们在下面引入了时间的处理方法， 也即接收的数据够多或者超时，就读取一次数据， 
然后根据当前的状态决定执行，如果没有连接上，就把所有数据当做AT命令处理， 如果连接
上了，就把数据送到对端。
*/

//uart 回调函数

static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events )
{
    (void)port;

    static uint32 old_time;     //老时间
    static uint32 old_time_data_len = 0;     //老时间是的数据长度    
    uint32 new_time;            //新时间
    bool ret;
    uint8 readMaxBytes = SIMPLEPROFILE_CHAR6_LEN;
        
    if (/*(events & HAL_UART_RX_FULL) ||*/ (events & HAL_UART_RX_TIMEOUT) /*|| (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))*/)   //串口有数据
    {
        (void)port;
        uint8 numBytes = 0;
        
        uint8 *buffer ;
        
        numBytes = NPI_RxBufLen();           //读出串口缓冲区有多少字节
        buffer = osal_mem_alloc(numBytes/*SIMPLEPROFILE_CHAR6_LEN*/);
        NPI_ReadTransport(buffer,numBytes);    //释放串口数据
        NPI_WriteTransport(buffer,numBytes);
        osal_mem_free(buffer);
        /*
        if(numBytes == 0)
        {
            LCD_WRITE_STRING_VALUE( "ERROR: numBytes=", numBytes, 10, HAL_LCD_LINE_1 );
	     NPI_WriteTransport("ERROR: numBytes=0\r\n",19);
            old_time_data_len = 0;
            return;
        }
        if(old_time_data_len == 0)
        {
            old_time = osal_GetSystemClock(); //有数据来时， 记录一下
            old_time_data_len = numBytes;
        }
        else
        {
            // 注意: 未连接上时， 有些AT 命令比较长， 所以需要开辟较大的缓冲区
            //       连接上以后， 收到每一能发送的数据不超过 SIMPLEPROFILE_CHAR6_LEN 的字节数的限制
            //       因此，这里要限制一下
            if(!simpleBLE_IfConnected())
            {
               readMaxBytes = 22 ;    //这个值， 一般设置成 AT 命令中最长的字节数即可， (包含"\r\n" 计数)
            }
            else
            {
               readMaxBytes = SIMPLEPROFILE_CHAR6_LEN;
            }

            
            new_time = osal_GetSystemClock(); //当前时间
            if( (numBytes >= readMaxBytes) 
                || ( (new_time - old_time) > 600))//ms
            {
                uint8 sendBytes = 0;
                uint8 *buffer = osal_mem_alloc(SIMPLEPROFILE_CHAR6_LEN);

                if(!buffer)
                {
                    NPI_WriteTransport("FAIL", 4); 
                    return;
                }
                
                // 
                if(numBytes > readMaxBytes)
                {
                    sendBytes = readMaxBytes;
                }
                else
                {
                    sendBytes = numBytes;
                }

                if(!simpleBLE_IfConnected())
                {//还没连接上
                    //numBytes = NpiReadBuffer(buf, sizeof(buf));
                    //NpiClearBuffer();
                    NPI_ReadTransport(buffer,sendBytes);    //释放串口数据    ，读取串口数据
                    
                    if(sendBytes > 2 
                    && buffer[sendBytes-2] == '\r' 
                    && buffer[sendBytes-1] == '\n')
                    {//检测到 \r\n 结束的字符串， 表明是 AT 命令
                        ret = simpleBLE_AT_CMD_Handle(buffer, sendBytes);
                    }
                    else
                    {
                        ret = FALSE;
                    }
                    
                    if(ret == FALSE)
                    {
                        char strTemp[12];
                        //参数错误， 直接返回 "ERROR\r\n"， 不做任何参数更改
                        sprintf(strTemp, "ERROR\r\n");
                        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                    }
                }
                else
                {
                    if((GetBleRole() == BLE_ROLE_CENTRAL) && simpleBLEChar6DoWrite && simpleBLECentralCanSend )             
                    {
                        char strTemp[24];

                        NPI_ReadTransport(buffer,sendBytes);    //释放串口数据    

                        // 判断是否是查询 RSSI
                        if((sendBytes == 10) && str_cmp(buffer, "AT+RSSI?\r\n", 10))//AT+RSSI\r\n    
                        {
                            sprintf(strTemp, "OK+RSSI:%d\r\n", sys_config.rssi);
                            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));                                    
                        }
                        else
                        {                        
                            //LCD_WRITE_STRING_VALUE( "sendBytes=", sendBytes, 10, HAL_LCD_LINE_1 );                            
                            simpleBLE_UartDataMain(buffer,sendBytes);//发送数据到模块
                        }
                    }
                    else if((GetBleRole() == BLE_ROLE_PERIPHERAL) && simpleBLEChar6DoWrite2)                    
                    {
                        //LCD_WRITE_STRING_VALUE( "sendBytes=", sendBytes, 10, HAL_LCD_LINE_1 );
                        NPI_ReadTransport(buffer,sendBytes);    //释放串口数据   
                        NPI_WriteTransport(buffer,sendBytes);
                        //simpleBLE_UartDataMain(buffer,sendBytes);//发送数据到手机端
                    }
                    else
                    {
                        //丢弃数据， 否则就会拖慢cpu
                        NPI_ReadTransport(buffer,sendBytes);    //释放串口数据    
                    }
                }

                old_time = new_time;
                old_time_data_len = numBytes - sendBytes;

                osal_mem_free(buffer);
            }                
        }  */  
    }
}

void simpleBLE_UartDataMain(uint8 *buf, uint8 numBytes)
{
    if(GetBleRole() == BLE_ROLE_CENTRAL )//主机
    {          
        if(simpleBLEChar6DoWrite 
        && ( simpleBLECharHd6 != 0)
        && simpleBLECentralCanSend)               //写入成功后再写入
        {
            attWriteReq_t AttReq;   

            LCD_WRITE_STRING_VALUE( "s=", numBytes, 10, HAL_LCD_LINE_1 );
            
            simpleBLEChar6DoWrite = FALSE;

            AttReq.handle = simpleBLECharHd6;
            AttReq.len = numBytes;
            AttReq.sig = 0;
            AttReq.cmd = 0;
            osal_memcpy(AttReq.value,buf,numBytes);
            GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );
        }
        else
        {
            LCD_WRITE_STRING_VALUE( "simpleBLEChar6DoWrite=", simpleBLEChar6DoWrite, 10, HAL_LCD_LINE_1 );
            LCD_WRITE_STRING_VALUE( "simpleBLECharHd6=", simpleBLECharHd6, 10, HAL_LCD_LINE_1 );
        }
    }
    else//从机
    {
        if(simpleBLEChar6DoWrite2)               //写入成功后再写入
        {                
#if 0 // 这种速度慢 SimpleProfile_SetParameter           
            simpleBLEChar6DoWrite2 = FALSE;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6,numBytes, buf );
#else // 这种速度快 GATT_Notification
            static attHandleValueNoti_t pReport;
            pReport.len = numBytes;
            pReport.handle = 0x0035;
            osal_memcpy(pReport.value, buf, numBytes);
			
            GATT_Notification( 0, &pReport, FALSE );            
#endif            
        }            
        else
        {
            LCD_WRITE_STRING_VALUE( "simpleBLEChar6DoWrite=", simpleBLEChar6DoWrite2, 10, HAL_LCD_LINE_1 );
        }
    }
}
#endif

#if 1
// AT 命令处理
bool simpleBLE_AT_CMD_Handle(uint8 *pBuffer, uint16 length)
{
    bool ret = TRUE;
    char strTemp[64];
    uint8 i;
    uint8 temp8;  
    bool restart = FALSE;
 
    //NPI_WriteTransport((uint8*)pBuffer, length); 
    // 1、测试
    if((length == 4) && str_cmp(pBuffer, "AT\r\n", 4))//AT    
    {
        sprintf(strTemp, "OK\r\n");
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }
    // 打印设置的参数  for test only
    else if((length == 8) && str_cmp(pBuffer, "AT+ALL\r\n", 8))//AT    
    {
        PrintAllPara();
    }
    // 2、查询、设置波特率
    else if((length == 10) && str_cmp(pBuffer, "AT+BAUD", 7))
    {
        /*
        发送：AT+BAUD2 
        返回：OK+Set:2 
        0---------9600 
        1---------19200 
        2---------38400 
        3---------57600 
        4---------115200
        */
        switch(pBuffer[7])
        {
        case '?':  //查询当前波特率
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.baudrate);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':  //查询设置新的波特率
            sys_config.baudrate = pBuffer[7] - '0';
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);    // 写所有参数        
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.baudrate);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        default:
            ret = FALSE;            
            break;
        }        
    }
    // 3、设置串口校验
    else if((length == 10) && str_cmp(pBuffer, "AT+PARI", 7))
    {
        /*
        Para 范围 0,1,2 
        0: 无校验
        1: EVEN 
        2: ODD 
        Default: 0 
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.parity);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
        case '2':
            sys_config.parity = pBuffer[7] - '0';        
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.parity);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }    
    // 4、设置停止位
    else if((length == 10) && str_cmp(pBuffer, "AT+STOP", 7))
    {
        /*
        Para: 0~1 
        0: 1 停止位
        1: 2 停止位
        Default: 0
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.stopbit);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
            sys_config.stopbit = pBuffer[7] - '0';  
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.stopbit);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }       
    // 5. 设置模块工作模式
    else if((length == 10) && str_cmp(pBuffer, "AT+MODE", 7))
    {
        /*
        Para: 0 ~ 1
        0: 开启串口透传模式
        1: 关闭串口透传模式
        2: iBeacon 广播模式
        Default: 0 
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.mode);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
        case '2':
            sys_config.mode = pBuffer[7] - '0';            
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.mode);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }          
    // 6、查询、设置设备名称
    else if((length >=10 && length <= 20) && str_cmp(pBuffer, "AT+NAME", 7))
    {
        /*
        Para1：设备名称
        最长 11 位数字或字母，
        含中划线和下划线，不建
        议用其它字符。
        Default：Microduino
        */
        //int nameLen = length - 7;
        
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%s\r\n", sys_config.name);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        default:
            osal_memset(sys_config.name, 0, sizeof(sys_config.name));
            osal_memcpy(sys_config.name, pBuffer + 7, length - 7);
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
            sprintf(strTemp, "OK+Set:%s\r\n", sys_config.name);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        }        
    }    
    //7. 恢复默认设置(Renew)
    else if((length == 10) && str_cmp(pBuffer, "AT+RENEW", 8))
    {
        sprintf(strTemp, "OK+RENEW\r\n");
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        
        simpleBLE_SetAllParaDefault(PARA_ALL_FACTORY);
        osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);  

        restart = TRUE;  //直接重启即可
    }
    //8. 模块复位，重启(Reset)
    else if((length == 10) && str_cmp(pBuffer, "AT+RESET", 8))
    {
        restart = TRUE;  //直接重启即可
    }
    // 9、查询、设置主从模式
    else if((length == 10) && str_cmp(pBuffer, "AT+ROLE", 7))
    {
        /*
        Para1: 0 ~ 1 
        1: 主设备
        0: 从设备
        Default: 0 
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.role);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
            temp8 = pBuffer[7] - '0';            
            if(temp8 == 0)
            {
              sys_config.role = BLE_ROLE_PERIPHERAL;
            }
            else
            {
              sys_config.role = BLE_ROLE_CENTRAL;
            }
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.role);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 


            restart = TRUE;  //直接重启即可
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }    
    // 10、 查询、设置配对密码
    else if(((length == 10) && str_cmp(pBuffer, "AT+PASS?", 8))
        || ((length == 15) && str_cmp(pBuffer, "AT+PASS", 7)))
    {
        /*
        Para1: 000000~999999 
        Default：000000
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+PASS:%s\r\n", sys_config.pass);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        default:
            osal_memcpy(sys_config.pass, pBuffer + 7, 6);
            sys_config.pass[6] = 0;
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
            sprintf(strTemp, "OK+Set:%s\r\n", sys_config.pass);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));
            break;
        }
    }   
    // 11、 设置模块鉴权工作类型
    else if((length == 10) && str_cmp(pBuffer, "AT+TYPE", 7))
    {
        /*
        Para: 0 ~ 1 
        0: 连接不需要密码
        1: 连接需要密码
        Default: 0 
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.type);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
            sys_config.type = pBuffer[7] - '0';            
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.type);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 


            restart = TRUE;  //直接重启即可
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }       
    // 12、 查询本机 MAC 地址
    else if((length >= 10) && str_cmp(pBuffer, "AT+ADDR?", 8))
    {        
        sprintf(strTemp, "OK+LADD:%s\r\n", sys_config.mac_addr);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }    
    // 13、 连接最后一次连接成功的从设备
    else if((length == 10) && str_cmp(pBuffer, "AT+CONNL", 8))
    {
        /*
        Para: L, N, E,F
        L:连接中、N:空地址
        E:连接错误、F:连接失败
        */
        uint8 para[4] = {'L','N','E','F'};
        int8 id = 0;
        
        if(sys_config.connect_mac_addr[0] != 0)
        {
            id = 0;
            simpleBLE_SetToConnectFlag(TRUE);
        }
        else
        {
            id = 3;
        }
        
        //if(sys_config.connl_status > 3) 
        //    sys_config.connl_status = 3;
        sprintf(strTemp, "AT+CONN%c\r\n", para[id]);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));
    }
    // 14、连接指定蓝牙地址的主设备或从设备
    else if((length == 20) && str_cmp(pBuffer, "AT+CON", 6))
    {
        /*
        Para1: MAC地址、
        如: 0017EA0923AE
        Para2: A, E, F
        A: 连接中
        E: 连接错误
        F: 连接失败
        */
        uint8 para[3] = {'A','E','F'};
        uint8 id = 0;
        
        osal_memcpy(sys_config.connect_mac_addr, pBuffer+6, MAC_ADDR_CHAR_LEN);
        osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            

        //if(simpleBLE_GetToConnectFlag == FALSE)
        {
            simpleBLE_SetToConnectFlag(TRUE);
            id = 0;        
        }
        //if(sys_config.connect_mac_status > 2) 
        //    sys_config.connect_mac_status = 2;        
        sprintf(strTemp, "AT+CONN%c\r\n", para[id]);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }
    // 15、 清除主设备配对信息
    else if((length == 10) && str_cmp(pBuffer, "AT+CLEAR", 8))
    {
        simpleBLE_SetAllParaDefault(PARA_PARI_FACTORY);
        osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
        //PrintAllPara();
        
        sprintf(strTemp, "OK+CLEAR\r\n");
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }
    // 16、查询成功连接过的从机地址
    else if((length == 10) && str_cmp(pBuffer, "AT+RADD?", 8))
    {
        for(i = 0; i<MAX_PERIPHERAL_MAC_ADDR; i++)
        {
            if(sys_config.ever_connect_mac_status[i][0] != 0)
            {
                sprintf(strTemp, "OK+RADD:%s\r\n", sys_config.ever_connect_mac_status[i]);
                NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            }
        }
    }
    // 17、 查询软件版本
    else if((length == 10) && str_cmp(pBuffer, "AT+VERS?", 8))
    {
        sprintf(strTemp, "%s\r\n", sys_config.verion);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }
    // 18、 设置主模式下尝试连接时间
    else if((length == 10) && str_cmp(pBuffer, "AT+TCON", 7))
    {
        /*
        指令	                应答	            参数
        查询：AT+TCON?	        OK+TCON:[para] 	
        设置：AT+TCON[para]	    OK+Set:[para] 	    Para: 000000～009999 
                                                    000000 代表持续连接，其
                                                    余代表尝试的毫秒数
                                                    Default:001000
        */
        if(pBuffer[7] == '?')
        {
            sprintf(strTemp, "%06d\r\n", sys_config.try_connect_time_ms);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        }
        else
        {             
            sys_config.try_connect_time_ms = 10000;//_atoi(pBuffer+7);
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
            sprintf(strTemp, "OK+Set:%06d\r\n", sys_config.try_connect_time_ms);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        }
    }    
    // 19、 读取 RSSI 信号值
    else if((length == 10) && str_cmp(pBuffer, "AT+RSSI?", 10))
    {
        sprintf(strTemp, "OK+RSSI:%d\r\n", sys_config.rssi);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));        
    }
    // 20、  改变模块发射信号强度
    else if((length == 10) && str_cmp(pBuffer, "AT+TXPW", 7))
    {
        /*
        指令	        应答	            参数
        查询：          AT+TXPW?	        OK+ TXPW:[para]	
        设置：          AT+TXPW[para]	    OK+Set:[para]	Para: 0 ~ 3
                                            0: 4dbm、1: 0dbm
                                            2: -6dbm、3: -23dbm
                                            Default: 0
        */
        if(pBuffer[7] == '?')
        {
            sprintf(strTemp, "AT+TXPW:%d\r\n", sys_config.txPower);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        }
        else
        {
            sys_config.txPower = pBuffer[7] - '0';
            if(sys_config.txPower > 3)
                sys_config.txPower = 0;
            osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.txPower);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

#if 1
        /*
#define LL_EXT_TX_POWER_MINUS_23_DBM                   0
#define LL_EXT_TX_POWER_MINUS_6_DBM                    1
#define LL_EXT_TX_POWER_0_DBM                          2
#define LL_EXT_TX_POWER_4_DBM                          3
        */
            // HCI_EXT_SetTxPowerCmd()是用来设置发射功率的. 有-23dbm, -6dbm, 0 dbm, +4dbm四个级别. 
            HCI_EXT_SetTxPowerCmd(sys_config.txPower);
#endif      

            restart = TRUE;  //直接重启即可
         }
    }        
    // 21、  改变模块作为ibeacon基站广播时间间隔
    else if((length == 10 || length == 15) && str_cmp(pBuffer, "AT+TIBE", 7))
    {
        /*
        指令	        应答	        参数
        查询：          AT+TIBE?	    OK+ TIBE:[para]	
        设置：          AT+TIBE[para]	OK+Set:[para]	Para: 000000～009999 
                                        000000 代表持续广播，其
                                        余代表尝试的毫秒数
                                        Default:000500
        */
        if(pBuffer[7] == '?')
        {
            sprintf(strTemp, "AT+TIBE:%06d\r\n", sys_config.ibeacon_adver_time_ms);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        }
        else
        {   
            sys_config.ibeacon_adver_time_ms = str2Num(pBuffer+7, 6);
            if(sys_config.ibeacon_adver_time_ms <= 9999)
            {
                osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
                sprintf(strTemp, "OK+Set:%06d\r\n", sys_config.ibeacon_adver_time_ms);
                NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

                restart = TRUE;  //直接重启即可
            }
        }
    }
    // 22、  设置模块工作类型 
    else if((length == 10) && str_cmp(pBuffer, "AT+IMME", 7))
    {
        /*
        指令	        应答	        参数
        查询：          AT+IMME?	    OK+Get:[para]	Para: 0~1
        设置：          AT+IMME[para]	OK+Set:[para]	Para: 0~1
                                        000000 代表持续广播，其
                                        0: 立即工作， 
                                        1: 等待AT+CON 或 AT+CONNL 命令
                                        Default:0
        */
        if(pBuffer[7] == '?')
        {
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.workMode);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        }
        else
        {   
            sys_config.workMode = str2Num(pBuffer+7, 1);
            if(sys_config.workMode <= 1)
            {
                osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config);            
                sprintf(strTemp, "OK+Set:%d\r\n", sys_config.workMode);
                NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            }

            restart = TRUE;  //直接重启即可
        }
    }    
    else
    {
        ret = FALSE;
    }

    if(restart)//如果该标志已设置，稍微延时后重启
    {
        Serial_Delay(100);      //设置参数后，适当延时， 以便上一次发送的数据正常发送出去
        HAL_SYSTEM_RESET(); 
    }
    
    return ret;
}
#endif

#if defined (AUTO_UART2UART)  
/*
很多朋友问我们， 如何实现把主机或从机上的传感器数据直接发送到对端并通过主机的串口
透传出去， 下面我们就能实现这个功能， 不过到底需要什么样的传感器， 以及什么样的数据
就需要你自己来组织了， 下面这个函数每100ms执行一次:
都可以把数据发送到对端， 对端通过串口透传出去。
下面给出一个样例: 实现把字符串发送到对方
*/
void simpleBLE_SendMyData_ForTest()
{
    static uint8 count_100ms = 0;
    uint8 numBytes;

    count_100ms++;
    if(count_100ms == 10)//本函数每100ms被执行一次， 计数10次就是1s
    {
        char strTemp[24] = {0};

        if((GetBleRole() == BLE_ROLE_CENTRAL) && simpleBLEChar6DoWrite && simpleBLECentralCanSend)               
        {
            sprintf(strTemp, "[%8ldms]Amo1\r\n", osal_GetSystemClock());
            //把你的数据组织到 strTemp， 就ok了, 注意不要超过 SIMPLEPROFILE_CHAR6_LEN 的大小
            //如果你发送的数据要超过 SIMPLEPROFILE_CHAR6_LEN， 那么最好的办法， 就是启动一个定时器，然后每定时器到， 就发送一段数据
            // 定时器的启动， 请参考       osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
            numBytes = (osal_strlen(strTemp) > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : osal_strlen(strTemp);            
            simpleBLE_UartDataMain((uint8*)strTemp, numBytes);
        }
        else if((GetBleRole() == BLE_ROLE_PERIPHERAL) && simpleBLEChar6DoWrite2)                    
        {
            sprintf(strTemp, "[%8ldms]Amo2\r\n", osal_GetSystemClock());
            //把你的数据组织到 strTemp， 就ok了, 注意不要超过 SIMPLEPROFILE_CHAR6_LEN 的大小
            //如果你发送的数据要超过 SIMPLEPROFILE_CHAR6_LEN， 那么最好的办法， 就是启动一个定时器，然后每定时器到， 就发送一段数据
            // 定时器的启动， 请参考       osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
            numBytes = (osal_strlen(strTemp) > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : osal_strlen(strTemp);
            simpleBLE_UartDataMain((uint8*)strTemp, numBytes);
        }

        count_100ms = 0;
    }
}
#endif

