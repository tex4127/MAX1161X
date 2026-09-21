#include <Arduino.h>
#include <Wire.h>
#include <MAX1161X.h>

int8_t MAX1161X_I2C_Read(uint8_t *buf, uint32_t len, void *intf_ptr);
int8_t MAX1161X_I2C_Write(const uint8_t *buf, uint32_t len, void *intf_ptr);
void MAX1161X_Delay_us(uint32_t period_us);

MAX1161X_Dev_t adc = {0};

union MAX1161X_intf_u{
  struct {
    uint8_t addr;
    TwoWire *m_i2c;
  } i2c;
};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  while(!Serial)
    ;
  adc.read = &MAX1161X_I2C_Read;
  adc.write = &MAX1161X_I2C_Write;
  adc.delay = &MAX1161X_Delay_us;
  max11615_init(&adc);
}

void loop() {
  
}

int8_t MAX1161X_I2C_Write(const uint8_t *buf, uint32_t len, void *intf_ptr){
  int8_t res = MAX1161X_STATUS_OK;
  MAX1161X_intf_u *comm = NULL;
  if (intf_ptr){
    comm = (MAX1161X_intf_u *)intf_ptr;
    comm->i2c.m_i2c->beginTransmission(comm->i2c.addr);
    comm->i2c.m_i2c->write(buf, len);
    if(comm->i2c.m_i2c->endTransmission()) res = MAX1161X_E_COM_FAIL;
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
    comm->i2c.m_i2c->readBytes(buf, 2);
  } else{
    res = MAX1161X_E_NULL_INTF;
  }
  return res;
}
void MAX1161X_Delay_us(uint32_t period_us){
  delayMicroseconds(period_us);
}
