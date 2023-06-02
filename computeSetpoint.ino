void computeSetPoint(int mode) {
  if (mode == 1) {
    setPointX = CENTRE_X;
    setPointY = CENTRE_Y;
  } else if (mode == 2) {
    unsigned int setPointCounter = (millis() / 2500) % 4;
    if (setPointCounter == 0) {
      setPointX = CENTRE_X - 150;
      setPointY = CENTRE_Y - 150;
    } else if (setPointCounter == 1) {
      setPointX = CENTRE_X + 150;
      setPointY = CENTRE_Y - 150;
    } else if (setPointCounter == 2) {
      setPointX = CENTRE_X + 150;
      setPointY = CENTRE_Y + 150;
    } else if (setPointCounter == 3) {
      setPointX = CENTRE_X - 150;
      setPointY = CENTRE_Y + 150;
    }
  } else if (mode == 3) {
    if (setPointDegrees_1 == 15) {
      setPointDegrees_1 = 0;
      setPointDegrees += 3;
    }
    if (setPointDegrees >= 360) {
      setPointDegrees = 0;
    }
    setPointX = CENTRE_X + 180 * sin(DEG_TO_RAD * setPointDegrees);
    setPointY = CENTRE_Y + 180 * cos(DEG_TO_RAD * setPointDegrees);
  }
}