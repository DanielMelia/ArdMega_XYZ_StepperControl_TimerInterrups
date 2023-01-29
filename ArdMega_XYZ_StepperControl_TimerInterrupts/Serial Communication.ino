void readCommand() {
  int ind1, ind2, ind3, ind4, ind5, ind6;
  //double x_val;
  if (inString.length() > 0) {
    char fisrt_char = inString.charAt(0);
    switch (fisrt_char) {

      case 'D':
        control_mode = MAN;
        set_delay();
        break;

      case 'M':
        mov_step_num();
        break;

      case 'G':
        mov2_step_pos();
        break;

      case 'I':

        control_mode = STP;

        ind1 = inString.indexOf(':');
        ind2 = inString.indexOf(';', ind1 + 1);
        curr_x = inString.substring(ind1 + 1, ind2).toFloat();

        ind3 = inString.indexOf(';', ind2 + 1);
        curr_y = inString.substring(ind2 + 1, ind3).toFloat();

        ind4 = inString.indexOf(';', ind3 + 1);
        curr_z = inString.substring(ind3 + 1, ind4).toFloat();

        pos_init = true;
        inString = "";

        double th1;
        double th2;

        inverseKinematics(th1, th2);  // Calculate initial joint angles (rad)

        steppers[1].pos = th1;
        steppers[2].pos = th2;
        Serial.println("I:" + String(0) + ";" + String(steppers[1].pos) + ";" + String(steppers[2].pos) + ";");

        break;

      case 'T':

        if (pos_init == true) {
          control_mode = TRAJ;

          ind1 = inString.indexOf(':');
          ind2 = inString.indexOf(';', ind1 + 1);
          trg_x = inString.substring(ind1 + 1, ind2).toFloat();

          ind3 = inString.indexOf(';', ind2 + 1);
          trg_y = inString.substring(ind2 + 1, ind3).toFloat();

          ind4 = inString.indexOf(';', ind3 + 1);
          trg_z = inString.substring(ind3 + 1, ind4).toFloat();

          ind5 = inString.indexOf(';', ind4 + 1);
          tf = inString.substring(ind4 + 1, ind5).toFloat();

          ind6 = inString.indexOf(';', ind5 + 1);
          ta = inString.substring(ind5 + 1, ind6).toFloat();

          ti = tf / vec_length;
          interval = (2 * clock_freq * ti) / prescaler;
          acc_time = (int)(ta / ti);

          inString = "";
          traj_calcXY();
          traj_calcZ();
        }

        break;

      case 'Z':
        control_mode = STP;
        steppers[0].curr_steps = 0;
        steppers[1].curr_steps = 0;
        steppers[2].curr_steps = 0;
        inString = "";
        break;
    }
  }
}

//Set Directions
void setDirections() {
  if (inString.charAt(1) == '1') { //If number of z-steps is negative
    Z_DIR_DOWN;
    steppers[0].dir = -1;
  }
  else {
    Z_DIR_UP;
    steppers[0].dir = 1;
  }
  if (inString.charAt(7) == '1') {
    TH1_DIR_CW;
    steppers[1].dir = -1;
  }
  else {
    TH1_DIR_CCW;
    steppers[1].dir = 1;
  }
  if (inString.charAt(13) == '1') {
    TH2_DIR_CCW;
    steppers[2].dir = -1;
  }
  else {
    TH2_DIR_CW;
    steppers[2].dir = 1;
  }
  steppers[0].count = 0;
  steppers[1].count = 0;
  steppers[2].count = 0;
}

void set_delay() {
  String value = inString.substring(1, 6);
  steppers[0].t_delay = value.toInt();
  value = inString.substring(6, 11);
  steppers[1].t_delay = value.toInt();
  value = inString.substring(11, 16);
  steppers[2].t_delay = value.toInt();
  inString = "";
}

void mov_step_num() {
  control_mode = MAN;
  setDirections();
  String value = inString.substring(2, 7);
  steppers[0].move_steps = value.toInt();
  value = inString.substring(8, 13);
  steppers[1].move_steps = value.toInt();
  value = inString.substring(14, 19);
  steppers[2].move_steps = value.toInt();
  inString = "";
}

void mov2_step_pos() {
  control_mode = MAN;
  int steps;

  String value = inString.substring(2, 7);
  if (inString.charAt(1) == '1') {
    steps = -(value.toInt()) - steppers[0].curr_steps;
  }
  else {
    steps = value.toInt() - steppers[0].curr_steps;
  }
  if (steps < 0) {
    Z_DIR_DOWN;
    steppers[0].dir = -1;
  }
  else {
    Z_DIR_UP;
    steppers[0].dir = 1;
  }
  steppers[0].move_steps = abs(steps);

  value = inString.substring(8, 13);
  if (inString.charAt(7) == '1') {
    steps = -(value.toInt()) - steppers[1].curr_steps;
  }
  else {
    steps = value.toInt() - steppers[1].curr_steps;
  }
  if (steps < 0) {
    TH1_DIR_CW;
    steppers[1].dir = -1;
  }
  else {
    TH1_DIR_CCW;
    steppers[1].dir = 1;
  }
  steppers[1].move_steps = abs(steps);

  value = inString.substring(14, 19);
  if (inString.charAt(13) == '1') {
    steps = -(value.toInt()) - steppers[2].curr_steps;
  }
  else {
    steps = value.toInt() - steppers[2].curr_steps;
  }
  if (steps < 0) {
    TH2_DIR_CCW;
    steppers[2].dir = -1;
  }
  else {
    TH2_DIR_CW;
    steppers[2].dir = 1;
  }
  steppers[2].move_steps = abs(steps);
  inString = "";
  steppers[0].count = 0;
  steppers[1].count = 0;
  steppers[2].count = 0;
}