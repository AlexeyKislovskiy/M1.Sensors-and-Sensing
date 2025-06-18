#include <ros.h>
#include <std_msgs/Char.h>
#include <LiquidCrystal.h>


const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


int x = 0;
int y = 0;


ros::NodeHandle nh;


void commandCallback(const std_msgs::Char& cmd) {
  char command = cmd.data;


  lcd.setCursor(x, y);
  lcd.print(' ');


  switch (command) {
    case 'W': y=(y-1)%2; break;          
    case 'S': y=(y+1)%2; break;          
    case 'A': x = (x == 0) ? 15 : x - 1;; break;          
    case 'D': x=(x+1)%16; break;         
  }

  
  lcd.setCursor(x, y);
  lcd.print('X');
}


ros::Subscriber<std_msgs::Char> sub("/computer/lcd/cmd", &commandCallback);

void setup() {
  
  lcd.begin(16, 2);  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print('X');

  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
