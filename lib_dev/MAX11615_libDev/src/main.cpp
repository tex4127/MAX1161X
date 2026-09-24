#include <Arduino.h>
#include <Wire.h>
#include <MAX1161X.h>
#include <IIR_Filter.h>

//#define __DEBUG__
//#define __TESTING__

#define NUM_ICH 8
#define IIR_ALPHA 0.2f

union MAX1161X_intf_u{
  struct {
    uint8_t addr;
    TwoWire *m_i2c;
  } i2c;
};

int8_t MAX1161X_I2C_Read(uint8_t *buf, uint32_t len, void *intf_ptr);
int8_t MAX1161X_I2C_Write(const uint8_t *buf, uint32_t len, void *intf_ptr);
void MAX1161X_Delay_us(uint32_t period_us);

int8_t filters_init(IIR_Filter_t *f, uint32_t num_filters);

MAX1161X_Dev_t adc = {0};
IIR_Filter_t filters[NUM_ICH] = {0};
MAX1161X_intf_u adc_intf = {MAX11615_I2C_ADDR, &Wire1};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  while(!Serial)
    ;
  delay(100);
  Serial.printf("Starting MAX1161X Testing\n");
  filters_init(filters, NUM_ICH);
  adc_intf.i2c.m_i2c->begin();
  adc.intf_ptr = &adc_intf;
  adc.read = &MAX1161X_I2C_Read;
  adc.write = &MAX1161X_I2C_Write;
  adc.delay = &MAX1161X_Delay_us;
  int8_t api_res = max11615_init(&adc);
  if (MAX1161X_STATUS_OK != api_res){
    Serial.printf("MAX1161X Error %d\n", api_res);
    while(1)
      delay(100);
  }
  #ifdef __TESTING__
  Serial.printf("Setup Byte 0x%02x\nConfig Byte 0x%02x\n", adc.setup.byte, adc.config.byte);
  while(1)
    delay(100);
  #endif
  max1161x_setExternalVRef(1.024, &adc);
  Serial.printf("Chip found and configured\n");
}

void loop() {
  uint32_t st = millis();
  uint32_t c = 0;
  //int8_t api_res = MAX1161X_STATUS_OK;
  float volts[NUM_ICH] = {0};
  while(millis() - st < 1000){
      for (uint8_t i = 0; i < NUM_ICH; i++){
        if (MAX1161X_STATUS_OK == max1161x_readADC_singleEnded(0b0111, &volts[i], &adc)){
          IIR_Filter_Update(volts[i], &filters[i]);
        }
      }
      c++;
    ;
  }
  for (uint8_t i = 0; i < NUM_ICH; i++){
    //Serial.printf("%04f,", volts[i]);
    Serial.printf("%04f,", filters[i].out);
  }
  Serial.printf("%lu | 0x%02x 0x%02x\n", c, adc.setup.byte, adc.config.byte);
  //Serial.printf("%lu\n", c);
}

int8_t MAX1161X_I2C_Write(const uint8_t *buf, uint32_t len, void *intf_ptr){
  int8_t res = MAX1161X_STATUS_OK;
  MAX1161X_intf_u *comm = NULL;
  if (intf_ptr){
    comm = (MAX1161X_intf_u *)intf_ptr;
    comm->i2c.m_i2c->beginTransmission(comm->i2c.addr);
    comm->i2c.m_i2c->write(buf, len);
    if(comm->i2c.m_i2c->endTransmission()) res = MAX1161X_E_COM_FAIL;
    #ifdef __DEBUG__
    Serial.printf("MAX1161X_I2C_Write() -> ");
    for (uint32_t i = 0 ; i < len; i++) Serial.printf("0x%02x ", buf[i]);
    Serial.printf("\n");
    #endif
  } else{
    res = MAX1161X_E_NULL_INTF;
  }
  return res;
}

int8_t MAX1161X_I2C_Read(uint8_t *buf, uint32_t len, void *intf_ptr){
  int8_t res = MAX1161X_STATUS_OK;
  MAX1161X_intf_u *comm = NULL;
  if (intf_ptr){
    comm = (MAX1161X_intf_u *)intf_ptr;
    comm->i2c.m_i2c->requestFrom((int)comm->i2c.addr, 2);
    comm->i2c.m_i2c->readBytes(buf, len);
    #ifdef __DEBUG__
    uint16_t by = (buf[0]<<8) | buf[1];
    Serial.printf("MAX1161X_I2C_Read %04x | %lu | %lu\n", by, by, adc.conversionTime);
    #endif
  } else{
    res = MAX1161X_E_NULL_INTF;
  }
  return res;
}
void MAX1161X_Delay_us(uint32_t period_us){
  delayMicroseconds(period_us);
  //delay(1);
}

int8_t filters_init(IIR_Filter_t *f, uint32_t num_filters){
  if (NULL == f) return 0;
  for (uint32_t i = 0 ; i < num_filters; i++){
    IIR_Filter_Init(IIR_ALPHA, &f[i]);
  }
  return 1;
}
