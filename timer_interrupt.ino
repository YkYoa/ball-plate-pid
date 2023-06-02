void timer_interrupt() {

  
  if (Output1 >= 0) {
    if (t1 > 0.4) {
      analogWrite(PWM_1, 0);
    } else {
      digitalWrite(DC1_LEN, HIGH);
      digitalWrite(DC1_XUONG, LOW);
      analogWrite(PWM_1, abs(Output1*21.25));
    }
  } else {
    if (t1 < -0.4) {
      analogWrite(PWM_1, 0);
    } else {
      digitalWrite(DC1_LEN, LOW);
      digitalWrite(DC1_XUONG, HIGH);
      analogWrite(PWM_1, abs(Output1*21.25));
    }
  }

  if (Output2 >= 0) {
    if (t2 > 0.4) {
      analogWrite(PWM_2, 0);
    } else {
      digitalWrite(DC2_LEN, HIGH);
      digitalWrite(DC2_XUONG, LOW);
      analogWrite(PWM_2, abs(Output2*21.25));
    }
  } else {
    if (t2 < -0.4) {
      analogWrite(PWM_2, 0);
    } else {
      digitalWrite(DC2_LEN, LOW);
      digitalWrite(DC2_XUONG, HIGH);
      analogWrite(PWM_2, abs(Output2*21.25));
    }
  }

  if (Output3 >= 0) {
    if (t3 > 0.4) {
      analogWrite(PWM_3, 0);
    } else {
      digitalWrite(DC3_LEN, HIGH);
      digitalWrite(DC3_XUONG, LOW);
      analogWrite(PWM_3, abs(Output3*21.25));
    }
  } else {
    if (t3 < -0.4) {
      analogWrite(PWM_3, 0);
    } else {
      digitalWrite(DC3_LEN, LOW);
      digitalWrite(DC3_XUONG, HIGH);
      analogWrite(PWM_3, abs(Output3*21.25));
    }
  }
  setPointDegrees_1 += 1;
}