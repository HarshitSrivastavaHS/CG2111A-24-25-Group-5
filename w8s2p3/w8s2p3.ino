// When an Arduino first starts after being freshly programmed
// there might be stray characters in the serial buffer.
// This function purges those characters.
#include <stdint.h>
typedef struct
{
  // Uncomment the line below for Step 7
  char c;
  char iamdumb[3];
  int32_t x;
  int32_t y;

} TData;

TData test;

void purgeSerial()
{
  char ch;

  while(Serial.available())
    ch = Serial.read();
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  // Purge stray characters
  purgeSerial();

  // Ucomment following line for Step 7
  test.c = 'a';
  test.x = 6;
  test.y= 10;
}

void waitForStart()
{
  int quit=0;

  while(!quit)
  {
    // Wait for start character
    while(!Serial.available());
    char ch = Serial.read();

    quit = (ch == 's');    
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  // Nothing since we just want to send over one packet
  // of data.

  waitForStart();
  
  char theSize = (char) sizeof(TData);
  Serial.write(theSize);
  Serial.write((char *) &test, sizeof(TData));
  
  // Uncomment following line in Step 7
  test.c++;
  test.x++;

}
