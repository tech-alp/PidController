#include "pid.h"

#define MSG_MAX_LEN 20
volatile uint8_t m_packetBuffer[MSG_MAX_LEN];
volatile uint8_t m_packetBufferInd;
bool m_message;

volatile int measurementSpeed;
/*-----------------------------------------*/
MotorDriver pid = {3.2f, 0.4f, 5.f, 0, 130};

unsigned long currentTime;
unsigned long preTime;

float setPoint = 9.0f;
float minI = 0.f, maxI = 130.f;
float res, input = 15.f;


// Simulation parameters
float t = 0;
//float input = 1;
float sim_tick_ms = 1;
float sim_totalTime_ms = 8000;
float sim_impulseArriveTime_ms = 500;

/*-----------------------------------------*/
void init_ComHandler()
{
  m_message = false;
  m_packetBufferInd = 0;
  for (int i = 0; i < MSG_MAX_LEN; i++)
  {
    m_packetBuffer[i] = 0;
  }
}

void uart_init()
{
  Serial.begin(115200);
}

void comHandle_handlerPacket()
{
  switch (m_packetBuffer[1])
  {
    case 0:
      Serial.print(res);
      Serial.print(' ');
      Serial.println(measurementSpeed);
      break;
    case 1:
      pid.set_pGain(makeFloat(m_packetBuffer, 2));
      pid.set_iGain(makeFloat(m_packetBuffer, 6));
      pid.set_dGain(makeFloat(m_packetBuffer, 10));
      //input = makeFloat(m_packetBuffer, 14);
      //pid.set_SetPoint(input);
      Serial.print(res);
      Serial.print(' ');
      Serial.println(measurementSpeed);
      break;
    case 2:
      String str = String(pid.get_Kp()) + " " + String(pid.get_Ki()) + " " + String(pid.get_Kd()) + " " + String(measurementSpeed);
      Serial.println(str);
      break;
    default : break;
  }
}

void setup() {
  uart_init();
  init_ComHandler();
  pid.set_SetPoint(setPoint);
}

void loop() {
  currentTime = millis();
  measurementSpeed = analogRead(A0);
  measurementSpeed = map(measurementSpeed,0,1023,0,25);
  
    /*Serial.print(pid.get_Kp());
    Serial.print(' ');
    Serial.print(pid.get_Ki());
    Serial.print(' ');
    Serial.println(pid.get_Kd());
    //delay(400);*/
  if(currentTime - preTime > 100)
  {
    res = pid.updatePID(measurementSpeed,currentTime);
  }
  preTime = currentTime;

  //input -= res;
  if (m_message)
  {
    m_message = false;
    comHandle_handlerPacket();
  }
  delay(100);
}

void serialEvent()
{
  while (Serial.available())
  {
    volatile uint8_t incomingData = Serial.read();
    m_packetBuffer[m_packetBufferInd] = incomingData;
    m_packetBufferInd = m_packetBufferInd + 1;

    if (m_message)
    {
      //..
    }

    else
    {
      if (m_packetBufferInd > MSG_MAX_LEN)
      {
        //Protect the buffer overflow
        m_packetBufferInd = 0;
        m_message = false;
      }

      else if (m_packetBufferInd == m_packetBuffer[0])
      {
        m_packetBufferInd = 0;
        m_message = true;
      }
    }
  }
}


// ----------------------------------------------------------------------------
float makeFloat(uint8_t* buff, uint8_t offset)
{
  uint8_t temp[4];
  float rv;
  for (uint8_t i = 0; i < 4; ++i)
  {
    temp[i] = buff[offset + i];
  }
  memcpy(&rv, temp, 4);
  return rv;
}
