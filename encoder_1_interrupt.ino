//--------------------
// for encoder1
//--------------------
void doencoder1A(){  // interrupt 0 function
  if (digitalRead(encoder1PinA) == HIGH) {  // look for a low-to-high on channel A
    if (digitalRead(encoder1PinB) == LOW) { // check channel B to see which way encoder is turning
      encoder1Count = encoder1Count + 1;
    } 
    else {
      encoder1Count = encoder1Count - 1;
    }
  }
  else {                                     // must be a high-to-low edge on channel A
    if (digitalRead(encoder1PinB) == HIGH) { // check channel B to see which way encoder is turning  
      encoder1Count = encoder1Count + 1;
    } 
    else {
      encoder1Count = encoder1Count - 1;
    }
  }
  t1=encoder1Count*6.283/7392;
}

void doencoder1B(){  // interrupt 1 function
  if (digitalRead(encoder1PinB) == HIGH) {   // look for a low-to-high on channel B
    if (digitalRead(encoder1PinA) == HIGH) { // check channel A to see which way encoder is turning
      encoder1Count = encoder1Count + 1;
    } 
    else {
      encoder1Count = encoder1Count - 1;
    }
  }
  else {                                    // must be a high-to-low edge on on channel B
    if (digitalRead(encoder1PinA) == LOW) { // check channel B to see which way encoder is turning
      encoder1Count = encoder1Count + 1;
    } 
    else {
      encoder1Count = encoder1Count - 1;
    }
  }
  t1=encoder1Count*6.283/7392;
}


