mode=<n> set foc running mode  
>> n = 0 : speed mode, dual motor runing as same target speed；  
>> n = 1 : postion mode, dual motor runing as same target position;  
>> n = 2 : postion feedback mode, use another motor's position as input;  
>> n = 3 : postion ratchet mode, Motor0 as input, Motor1 runs like a ratchet;  

target=<val> set the target value for running mode;  
>> speed mode, val = [-100, 100] rad/s;
>> position mode, val = [-1024, 1024] radian;
