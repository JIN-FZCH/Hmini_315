#define ALEU 0X01
#define ALED 0X02
#define ELEU 0X04
#define ELED 0X08
#define THRU 0X10
#define THRD 0X20
#define RUDU 0X40
#define RUDD 0X80

#define EXU   0X0100
#define EXD   0X0200
#define INU   0X0400
#define IND   0X0800
#define ALT   0X1000
#define CTRL  0X2000
#define SHIFT 0X4000
#define ENTER 0X8000

void Receive_Command(uint16_t *buf) {
  uint16_t key, dkey;

  dkey = *buf;
  if (dkey == 0xf1fe)
  {
    dkey += buf[1];
    dkey += buf[2];
    dkey += buf[3];
    if (dkey == buf[4])
    {
      control.ale = *((int16_t*)buf + 1);
      control.ele = *((int16_t*)buf + 2);
      key = buf[3];
      dkey = ~key & keylast; //按下了按键
      dkey |= ~key & (ALT | CTRL | SHIFT); //这三个可以连压
      if (dkey & SHIFT)
      {

      } else if (dkey & CTRL)
      {

      } else if (dkey & ALT)
      {

      } else
      {
        if (dkey & THRD) { //按了下降一下
          if (control.d_depth < 1) {
            control.d_depth += 0.2;
          }
          else {
            control.d_depth = 1;
          }
        } else if (dkey & THRU) { //按了上升一下
          if (control.d_depth > 0) {
            control.d_depth -= 0.2;
          }
          else {
            control.d_depth = 0;
          }
        }
      }
      keylast = key;
    }
  }
}

uint8_t Ux_Receive(uint8_t *num, uint16_t *buf, uint8_t val) {
  uint8_t tmp = 0;

  switch (*num)
  {
    case 0: if (val == 0XAA)(*num)++; break;
    case 1: if (val == 0XAA)(*num)++; else (*num) = 0; break;
    case 2: if (val == 0XAA)(*num)++; else (*num) = 0; break;
    case 3: if (val == 0XAA)(*num)++; else (*num) = 0; break;
    case 4: if (val == 0XAA)(*num)++; else (*num) = 0; break;
    case 5: if (val == 0XFE)(*num)++; else if (val != 0XAA)(*num) = 0; break;
    case 6: if (val == 0XF1)(*num)++; else (*num) = 0; break;
    case 7: if (val == 10)(*num)++; else (*num) = 0; break;
    case 8: *((uint8_t*)buf) = val; (*num)++; break;
    case 9: *((uint8_t*)buf + 1) = val; (*num)++; break;
    case 10: *((uint8_t*)buf + 2) = val; (*num)++; break;
    case 11: *((uint8_t*)buf + 3) = val; (*num)++; break;
    case 12: *((uint8_t*)buf + 4) = val; (*num)++; break;
    case 13: *((uint8_t*)buf + 5) = val; (*num)++; break;
    case 14: *((uint8_t*)buf + 6) = val; (*num)++; break;
    case 15: *((uint8_t*)buf + 7) = val; (*num)++; break;
    case 16: *((uint8_t*)buf + 8) = val; (*num)++; break;
    case 17: *((uint8_t*)buf + 9) = val; (*num) = 0; tmp = 1; Receive_Command(buf); break;
  }
  return tmp;
}
