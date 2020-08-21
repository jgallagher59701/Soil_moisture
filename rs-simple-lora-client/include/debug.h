//
// Created by James Gallagher on 6/7/20.
//

#ifndef SOIL_SENSOR_DEBUG_H
#define SOIL_SENSOR_DEBUG_H

#if DEBUG
#define IO(x) \
    do {      \
        x;    \
    } while (0)
#else
#define IO(x)
#endif

#if DEBUG2
#define IO2(x) \
    do {       \
        x;     \
    } while (0)
#else
#define IO2(x)
#endif

#endif //SOIL_SENSOR_DEBUG_H
