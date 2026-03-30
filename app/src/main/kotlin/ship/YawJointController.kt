package dev.sebastianb.ballcatcher.app.ship

import com.pi4j.context.Context
import com.pi4j.io.gpio.digital.DigitalOutput
import com.pi4j.io.gpio.digital.DigitalState
import com.pi4j.io.gpio.digital.PullResistance
import com.pi4j.ktx.io.digital.digitalInput
import dev.sebastianb.ballcatcher.app.motor.IMotorControl
import dev.sebastianb.ballcatcher.app.motor.IMotorFeedback
import dev.sebastianb.ballcatcher.app.motor.IMotorUnit
import dev.sebastianb.ballcatcher.app.motor.MotorState
import kotlin.math.abs

// TODO: make this a bit cleaner, I'm not 100% happy with tick (we should have a general tick schedular class)
// moveToAngle isn't that good either, we should handle states so we know if a action is currently running
class YawJointController(
    private val pi4j: Context,
    private val pulsePin: Int = 21,
    private val directionPin: Int = 20,
    private val leftSwitchPin: Int = 17,
    private val rightSwitchPin: Int = 27,
    val maxAngle: Double = 360.0,
): IMotorUnit {

    override var motorState: MotorState = MotorState.Uninitialized

    override val motorControl: IMotorControl = HardwarePwmMotorControl()

    override val motorFeedback: IMotorFeedback = TwoSwitchEncoderFeedback()

    // Absolute step counts from homing (left is distance going left, right is distance going right)
    var leftSwitchSteps: Long = 0
        private set
    var rightSwitchSteps: Long = 0
        private set

    // Takes a 0.0-1.0 float representing position between left (0) and right (1) switches
    // After homing, motor is at right switch (steps=0), left switch is in the negative direction
    // Uses trapezoidal speed profile: ramp up, cruise, ramp down
    suspend fun moveToPosition(fraction: Float) {
        val f = fraction.coerceIn(0f, 1f)
        val totalRange = rightSwitchSteps
        val targetSteps = (totalRange * (f - 1.0)).toLong() // negative = toward left
        val currentSteps = motorFeedback.currentRawSteps
        val stepsToMove = abs(targetSteps - currentSteps)

        if (stepsToMove < 2) return

        val ctrl = motorControl as HardwarePwmMotorControl
        val goForward = targetSteps > currentSteps
        ctrl.startRampedMove(goForward)

        val minFreq = 50
        val maxFreq = 600
        val rampSteps = (stepsToMove / 4).coerceIn(10, 150) // ramp over ~25% of travel

        while (true) {
            val remaining = abs(targetSteps - motorFeedback.currentRawSteps)
            if (remaining < 2) break

            val traveled = stepsToMove - remaining
            // ramp up at start, ramp down at end, cruise in between
            val rampUpFactor = (traveled.toDouble() / rampSteps).coerceIn(0.0, 1.0)
            val rampDownFactor = (remaining.toDouble() / rampSteps).coerceIn(0.0, 1.0)
            val factor = minOf(rampUpFactor, rampDownFactor)
            ctrl.currentFreq = (minFreq + (maxFreq - minFreq) * factor).toInt().coerceIn(minFreq, maxFreq)

            kotlinx.coroutines.yield()
        }

        ctrl.stop()
        ctrl.targetAngle = motorFeedback.currentAngle.toFloat()
    }

    override fun update() {
        motorControl.tick()
    }

    inner class HardwarePwmMotorControl() : IMotorControl {
        private val pulse: DigitalOutput = pi4j.create(
            DigitalOutput.newConfigBuilder(pi4j)
                .address(pulsePin)
                .id("yaw-pulse")
                .shutdown(DigitalState.LOW)
                .initial(DigitalState.LOW)
                .build()
        )

        private val direction: DigitalOutput = pi4j.create(
            DigitalOutput.newConfigBuilder(pi4j)
                .address(directionPin)
                .id("yaw-direction")
                .shutdown(DigitalState.LOW)
                .initial(DigitalState.LOW)
                .build()
        )

        var targetAngle: Float = 0f
        private var isEnabled: Boolean = false
        private var isMovingTillSwitch: Boolean = false
        private var lastPulseTime = System.nanoTime()
        var currentFreq = 0

        override suspend fun calibrateHome() {
            motorState = MotorState.Homing

            continuousMoveTillSwitchLeft()
            leftSwitchSteps = abs(motorFeedback.currentRawSteps)
            val leftAngle = motorFeedback.currentAngle
            (motorFeedback as TwoSwitchEncoderFeedback).resetSteps()

            continuousMoveTillSwitchRight()
            rightSwitchSteps = abs(motorFeedback.currentRawSteps)
            val rightAngle = motorFeedback.currentAngle
            (motorFeedback as TwoSwitchEncoderFeedback).resetSteps()

            println("Homing complete — Left: $leftSwitchSteps steps ($leftAngle°), Right: $rightSwitchSteps steps ($rightAngle°)")
            motorState = MotorState.Idle
        }

        override fun setEnabled(enabled: Boolean) {
            this.isEnabled = enabled
            if (enabled) {
                if (motorState == MotorState.Uninitialized) {
                    motorState = MotorState.Idle
                }
            } else {
                stop()
            }
        }

        override fun stop() {
            currentFreq = 0
            pulse.low()
            isMovingTillSwitch = false
            if (isEnabled) motorState = MotorState.Idle
        }

        override fun tick() {
            if (!isEnabled || !motorFeedback.isOn) {
                if (currentFreq > 0) stop()
                return
            }

            if (isMovingTillSwitch) {
                // direction and freq are set by continuousMoveTillSwitch*, just keep pulsing
            } else {
                val currentAngle = motorFeedback.currentAngle
                val error = targetAngle - currentAngle

                if (abs(error) < 0.5) { // Increased threshold slightly for stability
                    if (motorState is MotorState.Moving) {
                        motorState = MotorState.Idle
                        currentFreq = 0
                        pulse.low()
                    }
                    return
                }

                if (motorState !is MotorState.Moving) {
                    motorState = MotorState.Moving(motorFeedback)
                }

                val dirState = if (error > 0) DigitalState.HIGH else DigitalState.LOW
                direction.state(dirState)

                val maxFreq = 1000
                val minFreq = 50
                val gain = 500
                currentFreq = (abs(error) * gain).toInt().coerceIn(minFreq, maxFreq)
            }

            if (currentFreq > 0) {
                val intervalNs = 1_000_000_000L / currentFreq
                val now = System.nanoTime()
                if (now - lastPulseTime >= intervalNs) {
                    // Manual pulse: High then Low
                    pulse.high()
                    val startPulse = System.nanoTime()
                    while (System.nanoTime() - startPulse < 5000) { }
                    pulse.low()
                    lastPulseTime = now
                    
                    // Update feedback
                    val isForward = direction.state() == DigitalState.HIGH
                    (motorFeedback as TwoSwitchEncoderFeedback).onStep(isForward)
                }
            }
        }

        fun startRampedMove(forward: Boolean) {
            isMovingTillSwitch = true
            direction.state(if (forward) DigitalState.HIGH else DigitalState.LOW)
            currentFreq = 50
            motorState = MotorState.Moving(motorFeedback)
        }

        override suspend fun moveToAngle(degrees: Float) {
            targetAngle = degrees
            // Suspend until we are in Idle state and near the target
            while (motorState !is MotorState.Idle || abs(targetAngle - motorFeedback.currentAngle) >= 0.5) {
                kotlinx.coroutines.yield()
            }
        }

        override suspend fun moveClockwise(degrees: Float) {
            moveToAngle(motorFeedback.currentAngle.toFloat() + degrees)
        }

        override suspend fun moveCounterClockwise(degrees: Float) {
            moveToAngle(motorFeedback.currentAngle.toFloat() - degrees)
        }

        override suspend fun continuousMoveTillSwitchLeft() {
            isMovingTillSwitch = true
            direction.state(DigitalState.LOW) // CCW = toward left switch
            currentFreq = 200
            motorState = MotorState.Moving(motorFeedback)
            while (!(motorFeedback as TwoSwitchEncoderFeedback).isAtLeftSwitch) {
                kotlinx.coroutines.yield()
            }
            stop()
        }

        override suspend fun continuousMoveTillSwitchRight() {
            isMovingTillSwitch = true
            direction.state(DigitalState.HIGH) // CC = toward right switch
            currentFreq = 200
            motorState = MotorState.Moving(motorFeedback)
            while (!(motorFeedback as TwoSwitchEncoderFeedback).isAtRightSwitch) {
                kotlinx.coroutines.yield()
            }
            stop()
        }


    }

    inner class TwoSwitchEncoderFeedback(
        override val maxAngle: Double = this.maxAngle
    ) : IMotorFeedback {
        private var _isOn: Boolean = true
        override val isOn: Boolean
            get() = _isOn

        private val leftSwitch = pi4j.digitalInput(leftSwitchPin) {
            id("yaw-left-switch")
            address(leftSwitchPin)
            pull(PullResistance.PULL_UP)
            debounce(50000L)
        }

        private val rightSwitch = pi4j.digitalInput(rightSwitchPin) {
            id("yaw-right-switch")
            address(rightSwitchPin)
            pull(PullResistance.PULL_UP)
            debounce(50000L)
        }

        override val isAtLeftSwitch: Boolean
            get() = leftSwitch.state() == DigitalState.HIGH

        override val isAtRightSwitch: Boolean
            get() = rightSwitch.state() == DigitalState.HIGH

        private var currentSteps: Long = 0

        private val gearRatio = (3/1)
        private val stepsPerRevolution: Double = 400.0 * gearRatio // Assuming 400 steps per revolution

//        private val magnetSensor = pi4j.digitalInput(magnetPin) {
//            id("magnet-sensor")
//            address(magnetPin)
//            pull(PullResistance.PULL_UP)
//            debounce(5000L)
//        }.apply {
//            onLow {
//                _isOn = !_isOn
//                println("Magnet sensor toggled! Now: $_isOn")
//            }
//        }

        fun onStep(isForward: Boolean) {
            if (isForward) currentSteps++ else currentSteps--
        }

        fun resetSteps() {
            currentSteps = 0
        }

        override val angularVelocity: Double
            get() = 0.0

        override val currentRawSteps: Long
            get() = currentSteps

        override val currentAngle: Double
            get() = (currentSteps.toDouble() / stepsPerRevolution) * 360.0

    }

}