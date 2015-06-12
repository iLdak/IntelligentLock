#ifndef SIMPLEBLE_H
#define SIMPLEBLE_H

#ifdef __cplusplus
extern "C"
{
#endif

//------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

//#define RELEASE_VER                      //����汾������

//------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
#define NPI_TIMEOUT_EVT             0x0008


#define     VERSION     "v1.3"


// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   100//������100ms

#define MAX_PERIPHERAL_MAC_ADDR                   5//����¼�Ĵӻ���ַ

#define MAC_ADDR_CHAR_LEN                       12//mac��ַ���ַ����� (һ���ֽڵ��������ַ�)


typedef enum
{
    PARA_ALL_FACTORY = 0,           //ȫ���ָ���������
    PARA_PARI_FACTORY = 1,          //�����Ϣ�ָ���������-�൱����������Ϣ
}PARA_SET_FACTORY;

typedef enum
{
    BLE_ROLE_PERIPHERAL = 0,        //�ӻ���ɫ
    BLE_ROLE_CENTRAL = 1,           //������ɫ    
}BLE_ROLE;

// Application states
enum
{
  BLE_STATE_IDLE,                    //������-����״̬
  BLE_STATE_CONNECTING,             //������...
  BLE_STATE_CONNECTED,              //��������
  BLE_STATE_DISCONNECTING,          //�Ͽ�������
  BLE_STATE_ADVERTISING             //�ӻ��㲥��
};

enum
{
  BLE_MODE_SERIAL,                   // ����͸��ģʽ ��Ĭ�ϡ�
  BLE_MODE_DRIVER,                   // ֱ��ģʽ        
  BLE_MODE_iBeacon,                  // iBeacon �㲥ģʽ
  BLE_MODE_MAX,
};

typedef struct 
{
  /*
        0---------9600 
        1---------19200 
        2---------38400 
        3---------57600 
        4---------115200
  */
    uint8 baudrate;                 // ������ 
    uint8 parity;                   //У��λ
    uint8 stopbit;                  //ֹͣλ 
    uint8 mode;                     //����ģʽ 0:͸�� �� 1: ֱ�� , 2: iBeacon
    uint8 name[12];                 //�豸����

    BLE_ROLE role;                  //����ģʽ  0: �ӻ�   1: ����

    uint8 pass[7];                  //����

    /*
    Para: 0 ~ 1 
    0: ���Ӳ���Ҫ����
    1: ������Ҫ����
    */
    uint8 type;                     //��Ȩģʽ

    
    uint8 mac_addr[13];            //����mac��ַ

    uint8 connl_status;            //�������һ�ε�״̬
    uint8 connect_mac_status;      //����ָ����ַ�ķ���״̬
    uint8 connect_mac_addr[13];    //ָ��ȥ���ӵ�mac��ַ

    //�����ɹ����ӹ��Ĵӻ���ַ
    uint8 ever_connect_mac_status[MAX_PERIPHERAL_MAC_ADDR][13];       

    uint8 verion[5];       //�汾��Ϣ v1.0 ~ v9.9

    /*
    Para: 000000��009999 
    000000 ����������ӣ���
    ������Եĺ�����
    Default:001000
    */
    uint16 try_connect_time_ms;           // ��������ʱ��
    int8 rssi;                              //  RSSI �ź�ֵ
    uint8 rxGain;                           //  ��������ǿ��
    uint8 txPower;                          //  �����ź�ǿ��
    uint16 ibeacon_adver_time_ms;         // �㲥���
    uint8 workMode;                        //  ģ�鹤������  0: ���������� 1: �ȴ�AT+CON �� AT+CONNL ����
}SYS_CONFIG;
extern SYS_CONFIG sys_config;


extern void Serial_Delay(int times);

//flag: PARA_ALL_FACTORY:  ȫ���ָ���������
//flag: PARA_PARI_FACTORY: ��������Ϣ
extern void simpleBLE_SetAllParaDefault(PARA_SET_FACTORY flag);    
extern void simpleBLE_SaveAllDataToFlash();

extern void PrintAllPara(void);
extern bool simpleBLE_AT_CMD_Handle(uint8 *pBuffer, uint16 length);

extern void simpleBLE_NPI_init(void);

extern void UpdateRxGain(void);
extern void UpdateTxPower(void);

extern void LedSetState(uint8 onoff);
extern void simpleBle_SetRssi(int8 rssi);


extern BLE_ROLE GetBleRole();

extern uint32 str2Num(uint8* numStr, uint8 iLength);

extern void simpleBle_PrintPassword();

extern uint8* GetAttDeviceName();
extern void performPeriodicTask( void );

extern char *bdAddr2Str ( uint8 *pAddr );
extern void CheckKeyForSetAllParaDefault(void);

extern bool CheckIfUse_iBeacon();
extern bool simpleBle_GetIfNeedPassword();

extern void simpleBLE_SetToConnectFlag(bool bToConnect);
extern bool simpleBLE_GetToConnectFlag(uint8 *Addr);


extern uint32 Get_iBeaconAdvertisingInterral();
extern void simpleBLE_SetPeripheralMacAddr(uint8 *pAddr);
extern bool simpleBLE_GetPeripheralMacAddr(uint8 *pAddr);



extern uint8 simpleBLEState;
extern uint16 simpleBLECharHdl;
extern uint16 simpleBLECharHd6;
extern bool simpleBLEChar6DoWrite;
extern bool simpleBLEChar6DoWrite2;


#if defined (RELEASE_VER)
#define LCD_WRITE_STRING(str, option)                     
#define LCD_WRITE_SCREEN(line1, line2)                    
#define LCD_WRITE_STRING_VALUE(title, value, format, line)

#if defined (HAL_LCD)
#undef HAL_LCD
#define HAL_LCD FALSE 
#endif

#else
// LCD macros
#if HAL_LCD == TRUE
#define LCD_WRITE_STRING(str, option)                       HalLcdWriteString( (str), (option))
#define LCD_WRITE_SCREEN(line1, line2)                      HalLcdWriteScreen( (line1), (line2) )
#define LCD_WRITE_STRING_VALUE(title, value, format, line)  HalLcdWriteStringValue( (title), (value), (format), (line) )
#else
#define LCD_WRITE_STRING(str, option)                     
#define LCD_WRITE_SCREEN(line1, line2)                    
#define LCD_WRITE_STRING_VALUE(title, value, format, line)
#endif
#endif





extern uint8 simpleBLETaskId;               // ��������
extern uint8 simpleBLEState;
extern uint16 simpleBLECharHdl;
extern uint16 simpleBLECharHd6;
extern bool simpleBLECentralCanSend;
extern bool simpleBLEChar6DoWrite;
extern uint8 simpleBLEPeripheral_TaskID;        // �ӻ�����






#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLE_H */
