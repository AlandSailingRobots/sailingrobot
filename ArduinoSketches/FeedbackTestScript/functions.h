int calcArraySize(float stepLen, float maxAngle){
  
  float siz;
  float  x = (int)maxAngle % (int)stepLen;
  if (x == 0){
    siz = 1 + 2* ( maxAngle/stepLen);
  } else{
  
  siz = 1 + 2* ( maxAngle/stepLen + (stepLen-x)/stepLen);
  }
  return (int)siz;

}
