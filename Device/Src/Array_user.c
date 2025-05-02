//
// Created by ustc on 24-9-19.
//

#include "Array_user.h"

void rescaleArray(float *array, int size, float scale_factor)
{
    for (int i = 0; i < size; ++i)
    {
        array[i] *= scale_factor;
    }
}
