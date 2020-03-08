/*
  Timebase callback
  This example shows how to configure HardwareTimer to execute a callback at regular interval.
  Callback toggles pin.
  Once configured, there is only CPU load for callbacks executions.
*/


#define pin  13


void Update_IT_callback(HardwareTimer*)
{ // Toggle pin. 10hz toogle --> 5Hz PWM
  digitalWrite(pin, !digitalRead(pin));
}
#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif

HardwareTimer *MyTim = new HardwareTimer(Instance);

void setup()
{


  // Instantiate HardwareTimer object. Thanks to 'new' instanciation, HardwareTimer is not destructed when setup() function is finished.

  // configure pin in output mode
  pinMode(pin, OUTPUT);

  MyTim->setMode(2, TIMER_OUTPUT_COMPARE);  // In our case, channekFalling is configured but not really used. Nevertheless it would be possible to attach a callback to channel compare match.
  MyTim->setOverflow(1000000, MICROSEC_FORMAT); // 10 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
}

long timer1 = millis();
int i = 0;
bool toggle;
void loop()
{
  /* Nothing to do all is done by hardware. Even no interrupt required. */
  if ((timer1 + 3000) < millis()) {
    i = i + 1;
    //digitalWrite(pin, !digitalRead(pin));
    if (toggle){
    MyTim->setOverflow((1000000), MICROSEC_FORMAT);
    } else {
    MyTim->setOverflow((20000), MICROSEC_FORMAT);  
    }
    toggle = !toggle;
    timer1 = millis();
  }
}
