#include <SPI.h>

char buf [100];
volatile byte pos;
volatile boolean process_it;

void setup (void)
{
  Serial.begin (9600);

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);

  // 设置为接收状态
  SPCR |= _BV(SPE);

  // 准备接受中断
  pos = 0;   // 清空缓冲区
  process_it = false;

  // 开启中断
  SPI.attachInterrupt();
}


// SPI 中断程序
ISR (SPI_STC_vect)
{
  byte c = SPDR;  // 从 SPI 数据寄存器获取数据
  if (pos < sizeof(buf))
  {
    buf [pos++] = c;
    if (c == '\n')
      process_it = true;
  }
}

void loop (void)
{
  if (process_it)
  {
    buf [pos] = 0;  
    Serial.println (buf);
    pos = 0;
    process_it = false;
  } 
}
