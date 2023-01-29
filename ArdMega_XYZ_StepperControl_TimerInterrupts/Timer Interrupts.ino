void initialiseInterrupts() {

  if (control_mode == MAN) {
    for (int i = 0; i < NUM_STEPPERS; i++) {
      if  (steppers[i].move_steps != 0) {
        steppers[i].finished = false;
      }
    }
    OCR1A = steppers[0].t_delay;  //Timer1 -->  Z axis
    OCR3A = steppers[1].t_delay;  //Timer3 -->  TH1 axis
    OCR4A = steppers[2].t_delay;  //Timer4  --> TH2 axis
  }
  else if (control_mode == TRAJ) {
    steppers[0].finished = false;
    steppers[1].finished = false;
    steppers[2].finished = false;
    steppers[0].count = 0;
    steppers[1].count = 0;
    steppers[2].count = 0;
    OCR1A = interval;  //Timer1 -->  Z axis
    OCR3A = interval;  //Timer3 -->  TH1 axis
    OCR4A = interval;  //Timer4  --> TH2 axis
    i1 = 0;
    i2 = 0;
    i3 = 0;
    state1 = LOW;
    state2 = LOW;
    state3 = LOW;
  }

  if (control_mode != STP) {
    TCNT1  = 0;
    TIMER1_INTERRUPTS_ON
    TCNT3  = 0;
    TIMER3_INTERRUPTS_ON
    TCNT4  = 0;
    TIMER4_INTERRUPTS_ON
  }


}
//-------------------------------------------------------------------
void waitUntilFinished() {
  while (TIMSK1 || TIMSK3 || TIMSK4);
}

//------------------------------------------INTERRUPT SERVICE ROUTINES--------------------------------------------//
ISR(TIMER1_COMPA_vect)
{

  switch (control_mode) {
    case MAN:
      if (steppers[0].finished == true) {
        TIMER1_INTERRUPTS_OFF
      }
      else
      {
        Z_TOOGLE_STEP;

        steppers[0].pulse_status = !steppers[0].pulse_status;
        if (steppers[0].pulse_status == HIGH) {
          steppers[0].count++;
          steppers[0].curr_steps = steppers[0].curr_steps + steppers[0].dir;
        }
        if ( steppers[0].count >= steppers[0].move_steps ) {
          steppers[0].finished = true;
        }
      }

      break;

    case TRAJ:
      if (m1_steps [i1] != 0) {
        count1 = (unsigned int)(interval / (abs(m1_steps [i1]) * 2));
        Z_TOOGLE_STEP;
        state1 = !state1;
        if (rem_steps1 > 0 && state1 == LOW) {
          rem_steps1 = rem_steps1 - 1;
          steppers[0].count++;
          steppers[0].curr_steps = steppers[0].curr_steps + steppers[0].dir;
          steppers[0].pos +=  c_z * steppers[0].dir;
        }
        else if (rem_steps1 < 0 && state1 == LOW) {
          rem_steps1 = rem_steps1 + 1;
          steppers[0].count++;
          steppers[0].curr_steps = steppers[0].curr_steps + steppers[0].dir;
          steppers[0].pos +=  c_z * steppers[0].dir;
        }
        if (rem_steps1 == 0) {      //If remaining steps is equal to 0
          i1++;                       //Increase index
          rem_steps1 = m1_steps [i1]; //and look for the next value in the steps array
          //Set the stepper motor direction:
          if (rem_steps1 > 0) {
            steppers[0].dir = 1;
            Z_DIR_UP;
          }
          else if (rem_steps1 < 0) {
            steppers[0].dir = -1;
            Z_DIR_DOWN;
          }
        }
      }
      //If no steps are needed at this time interval:
      else if (m1_steps [i1] == 0) {
        count1 = interval;           //Set the timer to wait for another whole time interval
        i1++;                       //Increase index
        rem_steps1 = m1_steps [i1]; //and look for the next value in the steps array
        //Set the stepper motor direction:
        if (rem_steps1 > 0) {
          steppers[0].dir = 1;
          Z_DIR_UP;
        }
        else if (rem_steps1 < 0) {
          steppers[0].dir = -1;
          Z_DIR_DOWN;
        }
      }
      OCR1A = count1;
      if (i1 >= vec_length) {
        TIMER1_INTERRUPTS_OFF
        //Serial.println(steppers[1].count);
      }
      break;

    case STP:
      TIMER1_INTERRUPTS_OFF
      break;
  }
}
//-------------------------------------------------------------------
ISR(TIMER3_COMPA_vect)
{
  switch (control_mode) {
    case MAN:
      if (steppers[1].finished == true) {
        TIMER3_INTERRUPTS_OFF
      }
      else
      {
        TH1_TOOGLE_STEP;
        steppers[1].pulse_status = !steppers[1].pulse_status;
        if (steppers[1].pulse_status == HIGH) {
          steppers[1].count++;
          steppers[1].curr_steps = steppers[1].curr_steps + steppers[1].dir;
        }
        if ( steppers[1].count >= steppers[1].move_steps ) {
          steppers[1].finished = true;
        }
      }
      break;

    case TRAJ:
      if (m2_steps [i2] != 0) {
        count2 = (unsigned int)(interval / (abs(m2_steps [i2]) * 2));
        TH1_TOOGLE_STEP;
        state2 = !state2;
        if (rem_steps2 > 0 && state2 == LOW) {
          rem_steps2 = rem_steps2 - 1;
          steppers[1].count++;
          steppers[1].curr_steps = steppers[1].curr_steps + steppers[1].dir;
          steppers[1].pos +=  c_xy * steppers[1].dir;
        }
        else if (rem_steps2 < 0 && state2 == LOW) {
          rem_steps2 = rem_steps2 + 1;
          steppers[1].count++;
          steppers[1].curr_steps = steppers[1].curr_steps + steppers[1].dir;
          steppers[1].pos +=  c_xy * steppers[1].dir;
        }
        if (rem_steps2 == 0) {      //If remaining steps is equal to 0
          i2++;                       //Increase index
          rem_steps2 = m2_steps [i2]; //and look for the next value in the steps array
          //Set the stepper motor direction:
          if (rem_steps2 > 0) {
            steppers[1].dir = 1;
            TH1_DIR_CW;
          }
          else if (rem_steps2 < 0) {
            steppers[1].dir = -1;
            TH1_DIR_CCW;
          }
        }
      }
      //If no steps are needed at this time interval:
      else if (m2_steps [i2] == 0) {
        count2 = interval;           //Set the timer to wait for another whole time interval
        i2++;                       //Increase index
        rem_steps2 = m2_steps [i2]; //and look for the next value in the steps array
        //Set the stepper motor direction:
        if (rem_steps2 > 0) {
          steppers[1].dir = 1;
          TH1_DIR_CW;
        }
        else if (rem_steps2 < 0) {
          steppers[1].dir = -1;
          TH1_DIR_CCW;
        }
      }
      OCR3A = count2;
      if (i2 >= vec_length) {
        TIMER3_INTERRUPTS_OFF
        //Serial.println(steppers[1].count);
      }
      break;

    case STP:
      TIMER3_INTERRUPTS_OFF
      break;
  }
}
//-------------------------------------------------------------------
ISR(TIMER4_COMPA_vect)
{
  switch (control_mode) {
    case MAN:
      if (steppers[2].finished == true) {
        TIMER4_INTERRUPTS_OFF
      }
      else
      {
        TH2_TOOGLE_STEP;
        steppers[2].pulse_status = !steppers[2].pulse_status;
        if (steppers[2].pulse_status == HIGH) {
          steppers[2].count++;
          steppers[2].curr_steps = steppers[2].curr_steps + steppers[2].dir;
        }
        if ( steppers[2].count >= steppers[2].move_steps ) {
          steppers[2].finished = true;
        }
      }
      break;


    case TRAJ:
      if (m3_steps [i3] != 0) {
        count3 = (unsigned int)(interval / (abs(m3_steps [i3]) * 2));
        TH2_TOOGLE_STEP;
        state3 = !state3;
        if (rem_steps3 > 0 && state3 == LOW) {
          rem_steps3 = rem_steps3 - 1;
          steppers[2].count++;
          steppers[2].curr_steps = steppers[2].curr_steps + steppers[2].dir;
          steppers[2].pos +=  c_xy * steppers[2].dir;
        }
        else if (rem_steps3 < 0 && state3 == LOW) {
          rem_steps3 = rem_steps3 + 1;
          steppers[2].count++;
          steppers[2].curr_steps = steppers[2].curr_steps + steppers[2].dir;
          steppers[2].pos +=  c_xy * steppers[2].dir;
        }
        if (rem_steps3 == 0) {      //If remaining steps is equal to 0
          i3++;                       //Increase index
          rem_steps3 = m3_steps [i3]; //and look for the next value in the steps array
          //Set the stepper motor direction:
          if (rem_steps3 > 0) {
            steppers[2].dir = 1;
            TH2_DIR_CCW;
          }
          else if (rem_steps3 < 0) {
            steppers[2].dir = -1;
            TH2_DIR_CW;
          }
        }
      }
      //If no steps are needed at this time interval:
      else if (m3_steps [i3] == 0) {
        count3 = interval;           //Set the timer to wait for another whole time interval
        i3++;                       //Increase index
        rem_steps3 = m3_steps [i3]; //and look for the next value in the steps array
        //Set the stepper motor direction:
        if (rem_steps3 > 0) {
          steppers[2].dir = 1;
          TH2_DIR_CCW;
        }
        else if (rem_steps3 < 0) {
          steppers[2].dir = -1;
          TH2_DIR_CW;
        }
      }
      OCR4A = count3;
      if (i3 >= vec_length) {
        TIMER4_INTERRUPTS_OFF
      }
      break;

    case STP:
      TIMER4_INTERRUPTS_OFF
      break;
  }


}