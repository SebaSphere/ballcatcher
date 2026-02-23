package dev.sebastianb.ballcatcher.app.ship

import com.pi4j.context.Context
import com.pi4j.io.gpio.digital.DigitalOutput
import com.pi4j.io.gpio.digital.DigitalState
import com.pi4j.io.gpio.digital.PullResistance
import com.pi4j.io.pwm.Pwm
import com.pi4j.ktx.io.digital.digitalInput
import com.pi4j.ktx.io.digital.onLow
import dev.sebastianb.ballcatcher.app.motor.IMotorControl
import dev.sebastianb.ballcatcher.app.motor.IMotorFeedback
import dev.sebastianb.ballcatcher.app.motor.IMotorUnit
import dev.sebastianb.ballcatcher.app.motor.MotorState
import kotlin.math.abs

// TODO: make this a bit cleaner, I'm not 100% happy with tick (we should have a general tick schedular class)
// moveToAngle isn't that good either, we should handle states so we know if a action is currently running
class YawJointController(
    private val pi4j: Context,
    private val pulsePin: Int = 20,
    private val directionPin: Int = 21,
    private val magnetPin: Int = 24,
    val maxAngle: Double = 360.0,
): IMotorUnit {

    override var motorState: MotorState = MotorState.Uninitialized

    override val motorControl: IMotorControl = HardwarePwmMotorControl()

    override val motorFeedback: IMotorFeedback = MagneticEncoderFeedback()

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

        private var targetAngle: Float = 0f
        private var isEnabled: Boolean = false
        private var isHoming: Boolean = false
        private var lastPulseTime = System.nanoTime()
        private var currentFreq = 0

        override fun calibrateHome() {
            isHoming = true
            motorState = MotorState.Homing
            currentFreq = 200
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
            isHoming = false
            if (isEnabled) motorState = MotorState.Idle
        }

        override fun tick() {
            if (!isEnabled || !motorFeedback.isOn) {
                if (currentFreq > 0) stop()
                return
            }

            if (isHoming) {
                if (motorFeedback.isAtLimitSwitch) {
                    (motorFeedback as MagneticEncoderFeedback).resetSteps()
                    stop()
                    return
                }
                direction.state(DigitalState.LOW)
                currentFreq = 200
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
                    (motorFeedback as MagneticEncoderFeedback).onStep(isForward)
                }
            }
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

    }

    inner class MagneticEncoderFeedback(
        override val maxAngle: Double = this.maxAngle
    ) : IMotorFeedback {
        private var _isOn: Boolean = false
        override val isOn: Boolean
            get() = _isOn

        private var currentSteps: Long = 0
        private val stepsPerRevolution: Double = 400.0 // Assuming 400 steps per revolution

        private val magnetSensor = pi4j.digitalInput(magnetPin) {
            id("magnet-sensor")
            address(magnetPin)
            pull(PullResistance.PULL_UP)
            debounce(5000L)
        }.apply {
            onLow {
                _isOn = !_isOn
                println("Magnet sensor toggled! Now: $_isOn")
            }
        }

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

        override val isAtLimitSwitch: Boolean
            get() = magnetSensor.isLow
    }

}