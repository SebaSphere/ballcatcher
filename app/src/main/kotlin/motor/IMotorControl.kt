package dev.sebastianb.ballcatcher.app.motor

interface IMotorControl {
    // sets the positional data of any rotational data to 0 (for homing)
    fun calibrateHome()
    fun setEnabled(enabled: Boolean)
    fun stop()
    // the tick should control the motor state and intensity, we could look into "Hardware PWM"
    // we should use this state to ramp up the state for hardware PMW
    // article on linear motor control: https://www.eetimes.com/linear-motor-control-without-the-math/
    fun tick()
}