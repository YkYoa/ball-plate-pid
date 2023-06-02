//--------------------
// for encoder2
//--------------------
void doencoder2A(){  // interrupt 0 function
  if (digitalRead(encoder2PinA) == HIGH) {  // look for a low-to-high on channel A
    if (digitalRead(encoder2PinB) == LOW) { // check channel B to see which way encoder is turning
      encoder2Count = encoder2Count + 1;
    } 
    else {
      encoder2Count = encoder2Count - 1;
    }
  }
  else {                                     // must be a high-to-low edge on channel A
    if (digitalRead(encoder2PinB) == HIGH) { // check channel B to see which way encoder is turning  
      encoder2Count = encoder2Count + 1;
    } 
    else {
      encoder2Count = encoder2Count - 1;
    }
  }
  t2=encoder2Count*6.283/7392;
}

void doencoder2B(){  // interrupt 1 function
  if (digitalRead(encoder2PinB) == HIGH) {   // look for a low-to-high on channel B
    if (digitalRead(encoder2PinA) == HIGH) { // check channel A to see which way encoder is turning
      encoder2Count = encoder2Count + 1;
    } 
    else {
      encoder2Count = encoder2Count - 1;
    }
  }
  else {                                    // must be a high-to-low edge on on channel B
    if (digitalRead(encoder2PinA) == LOW) { // check channel B to see which way encoder is turning
      encoder2Count = encoder2Count + 1;
    } 
    else {
      encoder2Count = encoder2Count - 1;
    }
  }
  t2=encoder2Count*6.283/7392;
}


