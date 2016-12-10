#include "util.h"

float diffAngleDegrees(float prevAngle, float newAngle){
  float diff = newAngle - prevAngle;
  if (diff > 180.0)
    diff -= 360.0;
  if (diff < -180.0)
    diff += 360.0;
  return diff;

  /*float diff = fmod(newAngle - prevAngle, 360.0);
  if (diff < 0.0 )  diff  = diff + 360.0;
  if (diff > 180.0) diff  = diff - 360.0;
  return (-diff);*/

  /*if(prevAngle - newAngle <= 180){
    return prevAngle - newAngle;
  }
  return (newAngle+360) - prevAngle;*/
}