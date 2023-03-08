Arduino Thermocycle Serial Command Handling
===========================================

This is a simple Arduino project that allows you to control a thermocycler through serial commands. You can start, stop, preheat, and cooldown the heat block, as well as set the PID tune values and the temperature programs.

Commands
--------

Here are the available commands and their descriptions:

### Start Program

Start the thermocycling program.

```
START
```

### Stop Program

Stop the current running program.

```
STOP
```

### Preheat Heat Block

Preheat the heat block with the default temperature of 90°C.

```
PRE_HEAT
```

### Set Preheat Temperature

Set the preheat temperature to the specified temperature.

```
PRE_HEAT T=95
```

### Cooldown Heat Block

Cooldown the heat block.

```
COOLDOWN
```

### Get PID Value

Get the PID value.

```
GET_PID
```

### Set PID Tune Value

Set the PID tune value with the specified P, I, and D values.

```
PID_TUNE SET_PID P=2 I=1 D=1
```

### Set PID Tune Temperature

Set the PID tune temperature to the specified temperature.

```
PID_TUNE T=95
```

### Get Program List

Get the list of available temperature programs.

```
GET_PROGRAMS
```

### Set Program

Set the program with the specified step, temperature, duration, and ramp rate.

>S1 = DENATURATION
>S2 = ANNEALING
>S3 = EXTENTION
>F  = FINAL EXTENTION

```
SET_PROGRAM S1 T=95 D=50 RR=1
```




## Program States

The program state of a thermocycler can vary depending on the specific implementation, but here are some common program states:

1. Idle: The thermocycler is waiting for user input or for the temperature to stabilize before starting the next step.
2. Running: The thermocycler is currently executing a program step, such as denaturation, annealing, or extension.
3. Stopped: The thermocycler has completed all program steps and is stopped, awaiting further user input or a power cycle to start a new program.
4. Error: The thermocycler has encountered an error condition, such as a sensor failure or an over-temperature condition, and cannot continue until the issue is resolved.

There may be additional states depending on the specific features of the thermocycler, such as a pause state, a cooldown state, or a hold state.

## Thermal States

The thermal state of a thermocycler typically refers to the current temperature and thermal conditions of the thermocycler itself, as well as any samples or reagents that are being processed inside it. Some possible thermal states of a thermocycler include:

1. Heating: The thermocycler is actively raising the temperature of the sample block or chamber to a target setpoint temperature.
2. Cooling: The thermocycler is actively lowering the temperature of the sample block or chamber to a target setpoint temperature.
3. Holding: The thermocycler is maintaining the sample block or chamber at a constant target setpoint temperature, without actively heating or cooling it.
4. Equilibrating: The thermocycler is allowing the temperature to stabilize or equilibrate after a temperature change, before proceeding to the next step.
5. Error: The thermocycler has encountered an error condition, such as a sensor failure or a temperature overshoot, and is not able to maintain the desired thermal state.

The specific thermal states and temperature ranges used in a thermocycler will depend on the application and the types of samples or reagents being processed.

## Thermocycling Process Steps

Here's a general overview of the steps involved in a typical thermocycling process:

1. Denaturation: The initial step in many thermocycling protocols involves denaturing or separating the two strands of the DNA or RNA template. This is typically done by heating the sample to a high temperature, around 95°C to 98°C, for a short period of time, around 15 to 30 seconds. This step is necessary to allow the primers to bind to the single-stranded template.

2. Annealing: After denaturation, the sample is cooled to a temperature that allows the primers to anneal or bind to the template strands. The annealing temperature depends on the melting temperature (Tm) of the primers and the specific sequences being targeted. Typically, the annealing temperature is around 5°C to 10°C lower than the melting temperature of the primers, and the annealing time is around 20 to 30 seconds.

3. Extension: Once the primers are annealed to the template, the polymerase enzyme can extend the primers and synthesize new DNA strands. This is typically done at a slightly higher temperature than the annealing step, around 72°C to 74°C, and for a longer period of time, around 30 seconds to 1 minute per kilobase of DNA being amplified.

4. Repeat: Steps 1-3 are repeated for a number of cycles, typically between 20 and 40 cycles, to amplify the DNA or RNA target sequence exponentially.

5. Final Extension: After the final amplification cycle, a final extension step is performed to ensure
