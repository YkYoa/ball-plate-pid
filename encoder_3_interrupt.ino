//--------------------
// for encoder3
//--------------------
void doencoder3A(){  // interrupt 0 function
  if (digitalRead(encoder3PinA) == HIGH) {  // look for a low-to-high on channel A
    if (digitalRead(encoder3PinB) == LOW) { // check channel B to see which way encoder is turning
      encoder3Count = encoder3Count + 1;
    } 
    else {
      encoder3Count = encoder3Count - 1;
    }
  }
  else {                                     // must be a high-to-low edge on channel A
    if (digitalRead(encoder3PinB) == HIGH) { // check channel B to see which way encoder is turning  
      encoder3Count = encoder3Count + 1;
    } 
    else {
      encoder3Count = encoder3Count - 1;
    }
  }
  t3=encoder3Count*6.283/7392;
}

void doencoder3B(){  // interrupt 1 function
  if (digitalRead(encoder3PinB) == HIGH) {   // look for a low-to-high on channel B
    if (digitalRead(encoder3PinA) == HIGH) { // check channel A to see which way encoder is turning
      encoder3Count = encoder3Count + 1;
    } 
    else {
      encoder3Count = encoder3Count - 1;
    }
  }
  else {                                    // must be a high-to-low edge on on channel B
    if (digitalRead(encoder3PinA) == LOW) { // check channel B to see which way encoder is turning
      encoder3Count = encoder3Count + 1;
    } 
    else {
      encoder3Count = encoder3Count - 1;
    }
  }
  t3=encoder3Count*6.283/7392;
}


