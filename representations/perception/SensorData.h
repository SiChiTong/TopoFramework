#ifndef SENSORDATA_H
#define SENSORDATA_H

#include "kernel/Framework.h"
#include "math/Vector3.h"

REPRESENTATION(SensorData)
class SensorData : public SensorDataBase
{
  public:

    SensorData()
    {
    }

    Vector3<double> acc;
    Vector3<double> gyro;
};

#endif

