#ifndef UTIL_H
#define UTIL_H 

typedef union byteFloat {
    float F;
    unsigned char B[sizeof(float)];
} byteFloat;

typedef union byteInt {
    int I;
    unsigned char B[sizeof(int)];
} byteInt;

float diffAngleDegrees(float prevAngle, float newAngle);

#endif // util_H