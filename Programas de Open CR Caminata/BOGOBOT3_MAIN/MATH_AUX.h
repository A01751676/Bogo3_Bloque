#ifndef MATH_AUX
#define MATH_AUX

/*  MATHEMATICS AUXILIARY FUNCTIONS
 *   
 *  Available functions:
 * 
 *  double sign().....................Gets a number's sign.............................................................OK
 *  double atan2_()................Calculates atan2 math function...................................................OK
 *  double min_()..................Calculates min math function.....................................................OK
 *  double max_()..................Calculates max math function.....................................................OK
 *  
 */

double sign (double number) {
  if (number>=0) {return 1.0;}
  return -1.0;
}

double atan2_(double y, double x){
  if (x>0) {return atan(y/x);}
  if (x<0) {return atan(y/x)+ sign(y)*PI;}
  if(x==0 && y != 0) {return sign(y)*PI/2;} 
  return 0;
}

double min_(double a, double b){
  if (a < b){return a;}
  else {return b;}
}

double max_(double a, double b){
  if (a > b){return a;}
  else {return b;}
}
#endif
