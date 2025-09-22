# PID refactoring branch

This branch is for the development of a new PID system that has a more simple but still very flexible implementation.

## Examples

How to use new PID objects

### Create the object
The PID constructor only requires a PID Tuning object. The other arguments have defaults set.

``` C++
Mines::PIDTuning tuning;
tuning.kP = 1.0;
tuning.kI = 1.0;
tuning.kD = 1.0;

Mines::PID pid(tuning, 0.05, 10, 1000, msec);
```

### Setting the target
Before you use the PID you need to set a target for it to go to this example will use a motor and will spin it to 1000 ticks

```C++
pros::Motor motor(1);
double pos = motor.get_position();

pid.setTarget(pos);
```
### Using the PID object
The previously created PID object can be used in a way similar to the following the loop will exit once the PID reaches it's position for the goal time or if it times out.
``` C++
while(pid)
{
    motor.move(pid.update(motor.get_position()));
    pros::delay(20);
}
````