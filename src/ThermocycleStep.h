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

    String getName() const
    {
        return name;
    }

    int getDuration() const
    {
        return duration;
    }

    double getTemperature() const
    {
        return temperature;
    }

    double getRampRate() const
    {
        return rampRate;
    }

    void setName(const String &name)
    {
        this->name = name;
    }

    void setDuration(int duration)
    {
        this->duration = duration;
    }

    void setTemperature(double temperature)
    {
        this->temperature = temperature;
    }

    void setRampRate(double rampRate)
    {
        this->rampRate = rampRate;
    }
};

#endif