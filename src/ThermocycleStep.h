#ifndef THERMOCYCLESTEP_H
#define THERMOCYCLESTEP_H

#include <Arduino.h>

class ThermocycleStep
{
private:
    String name;
    int duration;
    double temperature;
    double rampRate;

public:
    ThermocycleStep(String name, double temperature, int duration, double rampRate)
    {
        this->name = name;
        this->duration = duration;
        this->temperature = temperature;
        this->rampRate = rampRate;
    }

    String getName()
    {
        return name;
    }

    int getDuration()
    {
        return duration;
    }

    double getTemperature()
    {
        return temperature;
    }

    double getRampRate()
    {
        return rampRate;
    }
};

#endif