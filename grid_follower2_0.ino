int centroid = 20 ;

int sensors[5] = {0,1,2,3,4};
int sensor_val;
int m1[2] = {5,6};
int m2[2] = {10,11};
int current_centroid;
float error = 0;
float correction;
float total_error = 0;
float prev_error = 0;
int i;
int mean_rpm = 125;
int kp = 60;
int ki = 0.1;
int kd = 0.1;
int goal;
int rgt_motor_rpm =0;
int lft_motor_rpm =0;
int junction = 0;
int goals[5][2] = {0,0, 2,3, 3,1, 4,4, 0,0};
int line =0;
int otn = 1;
int req_otn;
int temp_goal[2];
int temp1 = 0;
int no_of_coords = 5;
int j;
int detect =9;

void setup()
{
  for(i=0;i<2;i++)
  {
    pinMode(m1[i], OUTPUT);
    pinMode(m2[i],OUTPUT);
  }
  Serial.begin(9600);
}

void loop()
{
  int start = digitalRead(detect);
  delay(10);
  if(start == 1)
  {
    go_to_origin();
    grid_follow();
  }
  delay(10);
  
}

void go_to_origin()
{
  int o = junction;
  while(o == junction)
  {
    follow_line();
  }
  STOP();
  delay(20);
}

void grid_follow()
{
  
  
  for(j=0;j < no_of_coords ; j++)
  {
   compr_goals(goals[j][0],goals[j][1],goals[j+1][0],goals[j+1][1]);
 /*  Serial.println(goals[0][0]);
   Serial.println(goals[0][1]);
   Serial.println(goals[1][0]);
   Serial.println(goals[1][1]);
   Serial.println(goals[2][0]);
   Serial.println(goals[2][1]);
   Serial.println(goals[3][0]);
   Serial.println(goals[3][1]);*/
  
   
    goto_orientn(req_otn);
 
    if(otn == 1 || otn == 3)
    {
     goto_ycoordinate(temp_goal[1]);
     goto_xcoordinate(temp_goal[0]);
    }
    else
    {
     goto_ycoordinate(temp_goal[0]);
     goto_xcoordinate(temp_goal[1]);
    }
  
   STOP();
   delay(3000);
  
  }
}
 
  

void follow_line()
{
  sense_error();
  correctn();
  motor_control();
  temp1 = 0;
}
  
void sense_error()
{
  int sum = 0;
  int mass = 0;
   for(i=0;i<5;i++)
   {
    sensor_val = analogRead(sensors[i]);
    if(sensor_val <500)
    {
      sensor_val = 0;
    }
    else
    {
      sensor_val = 1;
    }
   
    mass += sensor_val;
    sum +=  sensor_val*i*10;
    
  }
  if(mass == 2 || mass == 3) line = 1;
  if(mass==5 && line ==1) 
    {
      junction++;
      line =0;
    }
   current_centroid = sum/mass;
   error = (current_centroid - centroid)/10 ;
}

void correctn()
{
   correction = kp*error + ki*total_error + kd*(error - prev_error);
   total_error += correction;
   prev_error = error;
  
}
void motor_control()
{
  rgt_motor_rpm = mean_rpm - correction;
   lft_motor_rpm = mean_rpm + correction;
   //Serial.println(rgt_motor_rpm);
   //Serial.println(lft_motor_rpm);
   //delay(1000);
    analogWrite(m1[0] , rgt_motor_rpm );
    analogWrite(m2[0] ,lft_motor_rpm);
   delay(2);
}
void rotate_right()
{
  if(temp1 == 0)
  {
    mov_forward();
    temp1 = 1;
  }
    analogWrite(m1[1] , 125 );
    analogWrite(m2[0] , 125);
    delay(500);
    analogWrite(m1[1] , 0 );
    analogWrite(m2[0] , 0 );
    delay(5);
    if(otn < 4)
    {
      otn++;
    }
    else
    {
      otn = 1;
    }
  
}
void rotate_left()
{
  if(temp1 == 0)
  {
    mov_forward();
    temp1 = 1;
  }
    analogWrite(m1[0] , 125 );
    analogWrite(m2[1] , 125 );
    delay(500);
    analogWrite(m1[0] , 0 );
    analogWrite(m2[1] , 0 );
    delay(5);
    if(otn == 1)
    {
      otn = 4;
    }
    else
    {
      otn--;
    }
    
  }

void mov_forward()
{
    analogWrite(m1[0] , 125 );
    analogWrite(m2[0] , 125);
    delay(300);
    analogWrite(m1[0] , 0);
    analogWrite(m2[0] , 0);
    delay(5);
}
void STOP()
{
  for(i=0;i<2;i++)
  {
    digitalWrite(m1[i] , LOW);
    digitalWrite(m2[i] , LOW);
  }
}
void goto_ycoordinate(int y)
{
  int y_temp = 0;
  
  junction =0;
  while(y_temp < y)
  {
  follow_line();
  y_temp = junction;
  //Serial.println(x_temp);
  }
  rotate_right();
  
}
void goto_xcoordinate(int x)
{
  int x_temp = 0;
  
  junction = 0;
  while(x_temp < x)
  {
    follow_line();
    x_temp = junction;
  }
}
void goto_orientn(int orientn)
{
  int temp;
  if(otn != orientn)
 { 
   if(orientn == 1 && otn == 4)
    {
      rotate_right();
    }
   else if(orientn == 4 && otn == 1)
    {
      rotate_left();
    }
    else if(orientn > otn)
     {
     temp = orientn - otn;
     while(temp>0)
     {
      rotate_right();
      temp--;
     }
     }
     else if(orientn < otn)
     {
     temp = otn - orientn;
     while(temp>0)
     {
      rotate_left();
      temp--;
      }
     }
 }
   
      
  }

void compr_goals(int g1 , int g2 , int g3 , int g4)
{
  if((g1 <= g3) && (g2 <= g4))
  {
    req_otn = 1;
    temp_goal[0] = g3 - g1; 
    temp_goal[1] = g4 - g2;
  }
    if((g1 <= g3) && (g2 >= g4))
  {
    req_otn = 2;
    temp_goal[0] = g3 - g1; 
    temp_goal[1] = g2 - g4;
  }
  
  if((g1 >= g3) && (g2 >= g4))
  {
    req_otn = 3;
    temp_goal[0] = g1 - g3; 
    temp_goal[1] = g2 - g4;
  }
   if((g1 >= g3) && (g2 <= g4))
  {
    req_otn = 4;
    temp_goal[0] = g1 - g3; 
    temp_goal[1] = g4 - g2;
  }

 
  

    
}
  
