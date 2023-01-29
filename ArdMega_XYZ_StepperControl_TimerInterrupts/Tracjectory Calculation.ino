void inverseKinematics(double &th1, double &th2) {
  double L0 = 200;
  double cQk = (sq(curr_x) + sq(curr_y) - 2 * sq(L0)) / (2 * sq(L0));
  double sQk = -sqrt(1 - sq(cQk));
  th1 = atan2(curr_y, curr_x) - atan2(L0 * sQk, L0 + L0 * cQk);
  th2 = atan2(sQk, cQk);
}

void traj_calcXY() {

  if (ta > tf / 2) {
    ta = tf / 2;
  }


  double rx [vec_length];
  double ry [vec_length];

  cartesian_path(rx, vec_length, acc_time, trg_x, curr_x);
  cartesian_path(ry, vec_length, acc_time, trg_y, curr_y);

  //cartesian_path(rx, ry, vec_length, acc_time);
  angular_trajectories(rx, ry, vec_length);
  //traj_digitalization(rx, ry, vec_length, min_step);

  traj_digitalization_single(rx, m2_steps, vec_length, min_step);
  traj_digitalization_single(ry, m3_steps, vec_length, min_step);

  //  for (int i = 0; i < vec_length; i++) {
  //    total_steps_x = total_steps_x + m2_steps[i];
  //    total_steps_y = total_steps_y + m3_steps[i];
  //  }
  //  distance_x = (min_step * total_steps_x) * RAD_TO_DEG;
  //  distance_y = (min_step * total_steps_y) * RAD_TO_DEG;
  //  displayvalues(rx);

}

void traj_calcZ() {

  if (ta > tf / 2) {
    ta = tf / 2;
  }

  double rz [vec_length];

  cartesian_path(rz, vec_length, acc_time, trg_z, curr_z);
  traj_digitalization_single(rz, m1_steps, vec_length, step_size_BS);
  //traj_digitalization(rx, ry, vec_length, min_step);

}

//-----------------------------------Cartesian Path--------------------------------------------------------//
void cartesian_path( double r1 [], int tot_steps, int acc_steps, double trg, double curr) {

  double v = (trg - curr) / (tf - ta);
  //Acceleration phase coefficients
  double a0 = curr;
  double a1 = v / (2 * ta);
  //Constant velocity phase coefficients
  double b1 = v;
  double b0 = curr - v * (ta / 2);
  //Deceleration phase coefficients
  double c0 = trg - (v * sq(tf)) / (2 * ta);
  double c1 = v * (tf / ta);
  double c2 = -v / (2 * ta);

  for (int i = 0; i < tot_steps; i++) {
    double t = tf * i / tot_steps;
    if (i <= acc_steps) { //i <=3
      r1 [i] = (a0 + a1 * sq(t));// * 1000;
    }
    else if (i > acc_steps && i <= (tot_steps - acc_steps)) { //i>3 & i<=7
      r1[i] = (b0 + b1 * t);// * 1000;
    }
    else if (i > (tot_steps - acc_steps) && i < tot_steps - 1) { //i>7 & i<=10
      r1[i] = (c0 + c1 * t + c2 * sq(t));// * 1000;
    }
    else { //i>10
      r1[i] = trg;// * 1000;
    }
  }
}

//----------------------------------------------------Angular Trajectories----------------------------------------------//
void angular_trajectories(double r1 [], double r2 [], int tot_steps) {
  for (int i = 0; i < tot_steps; i++) {
    double L = 200; // Arm segment length (mm)
    double cQk = (sq(r1 [i]) + sq(r2 [i]) - 2 * sq(L)) / (2 * sq(L));
    double sQk = -sqrt(1 - sq(cQk));
    r1[i] = atan2(r2[i], r1[i]) - atan2(L * sQk, L + L * cQk);
    r2[i] = atan2(sQk, cQk);
  }
}
//--------------------------------------------------Trajectory Digitalization-----------------------------------------//
// void traj_digitalization(double r1 [], double r2 [], int tot_steps, double min_step) {
//   for (int i = 0; i < tot_steps; i++) {
//     if (i == 0) {
//       //r1 [i] = r1 [i];
//       //r2 [i] = r2 [i];
//     }
//     else {
//       double th1_steps = (r1 [i] - r1 [i - 1]) / min_step;
//       double th2_steps = (r2 [i] - r2 [i - 1]) / min_step;
//       double quo1, rem1;
//       double quo2, rem2;
//       quotient_remainder(th1_steps, quo1, rem1);
//       quotient_remainder(th2_steps, quo2, rem2);

//       //Motor th1
//       if (th1_steps > 0.5) {
//         if (rem1 > 0.5) {
//           //r1 [i] = r1 [i - 1] + (quo1 + 1) * min_step;
//           m2_steps[i] = quo1 + 1;
//         }
//         else {
//           //r1 [i] = r1 [i - 1] + quo1 * min_step;
//           m2_steps[i] = quo1;
//         }
//       }
//       else if (th1_steps < -0.5) {
//         if (rem1 < -0.5) {
//           //r1 [i] = r1 [i - 1] + (quo1 - 1) * min_step;
//           m2_steps[i] = quo1 - 1;
//         }
//         else {
//           //r1 [i] = r1 [i - 1] + quo1 * min_step;
//           m2_steps[i] = quo1;
//         }
//       }
//       else {
//         //r1 [i] = r1 [i - 1];
//         m2_steps[i] = 0;
//       }

//       //Motor th2
//       if (th2_steps > 0.5) {
//         if (rem2 > 0.5) {
//           //r2 [i] = r2 [i - 1] + (quo2 + 1) * min_step;
//           m3_steps[i] = quo2 + 1;
//         }
//         else {
//           //r2 [i] = r2 [i - 1] + quo2 * min_step;
//           m3_steps[i] = quo2;
//         }
//       }
//       else if (th2_steps < -0.5) {
//         if (rem2 < -0.5) {
//           //r2 [i] = r2 [i - 1] + (quo2 - 1) * min_step;
//           m3_steps[i] = quo2 - 1;
//         }
//         else {
//           //r2 [i] = r2 [i - 1] + quo2 * min_step;
//           m3_steps[i] = quo2;
//         }
//       }
//       else {
//         //r2 [i] = r2 [i - 1];
//         m3_steps[i] = 0 ;
//       }
//     }
//   }
// }

void traj_digitalization_single(double r_in [], int r_out [], int tot_steps, double min_step) {
  for (int i = 0; i < tot_steps; i++) {
    if (i != 0) {
      double th1_steps = (r_in [i] - r_in [i - 1]) / min_step;
      double quo1, rem1;
      quotient_remainder(th1_steps, quo1, rem1);
      //Motor th1
      if (th1_steps > 0.5) {
        if (rem1 > 0.5) {
          r_out[i] = quo1 + 1;
        }
        else {
          r_out[i] = quo1;
        }
      }
      else if (th1_steps < -0.5) {
        if (rem1 < -0.5) {
          r_out[i] = quo1 - 1;
        }
        else {
          r_out[i] = quo1;
        }
      }
      else {
        r_out[i] = 0;
      }
    }
    else {
      r_out[i] = 0;
    }
  }
}

void quotient_remainder (double num, double &quo, double &rem) {
  int div_int = (int)num;
  quo = div_int;
  rem = num - div_int;
}