#include <Servo.h>
Servo myservo_x , myservo_y;
int servo_x = 90;
int servo_y = 70;
bool servo_run = true;
void servo_up(){
    servo_y += 1;
    if(servo_y>120)
        servo_y = 120;
    myservo_y.write(servo_y);
}
void servo_down(){
    servo_y -= 1;
    if(servo_y<40)
        servo_y = 40;
    myservo_y.write(servo_y);
}
void servo_right(){
    servo_x += 1;
    if(servo_x>180)
        servo_x = 180;
    myservo_x.write(servo_x);
}
void servo_left(){
    servo_x -= 1;
    if(servo_x<0)
        servo_x = 0;
    myservo_x.write(servo_x);
}
void servo_reset(){
    servo_x = 90;
    servo_y = 70;
    myservo_x.write(servo_x);
    myservo_y.write(servo_y);
}

void setup(){
    myservo_x.attach(9);
    myservo_y.attach(10);
    myservo_x.write(servo_x);
    myservo_y.write(servo_y);
    Serial.begin(9600);
    pinMode(2 , INPUT_PULLUP);
}
void loop(){
     if(digitalRead(2)==LOW){
         servo_run = !servo_run;
         delay(1000);
     }
     if(servo_run){
         char data = char(Serial.read());
         switch(data){
             case 'u':
                 servo_up();
             break;
             case 'd':
                 servo_down();
             break;
             case 'l':
                 servo_left();
             break;
             case 'r':
                 servo_right();
             break;
             case 's':
                 servo_reset();
             break;
         }
         Serial.write('1');
     }
     delay(100);
}
