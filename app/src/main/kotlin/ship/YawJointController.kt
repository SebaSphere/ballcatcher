package dev.sebastianb.ballcatcher.app.ship

import dev.sebastianb.ballcatcher.app.motor.IMotorControl
import dev.sebastianb.ballcatcher.app.motor.IMotorFeedback
import dev.sebastianb.ballcatcher.app.motor.IMotorUnit
import dev.sebastianb.ballcatcher.app.motor.MotorState

// TODO: will use the motor interface classes as inner classes for the concrete implementation, can refactor once it starts getting large
class YawJointController(
    val maxAngle: Double = 360.0,
): IMotorUnit {

    override val motorState: MotorState = MotorState.Uninitialized

    override val motorControl: IMotorControl = HardwarePwmMotorControl()

    override val motorFeedback: IMotorFeedback = MagneticEncoderFeedback()

    override fun update() {
        TODO("Not yet implemented")
    }

    inner class HardwarePwmMotorControl() : IMotorControl {
        override fun calibrateHome() {
            TODO("Not yet implemented")
        }

        override fun setEnabled(enabled: Boolean) {
            TODO("Not yet implemented")
        }

        override fun stop() {
            TODO("Not yet implemented")
        }

        override fun tick() {
            TODO("Not yet implemented")
        }

    }

    inner class MagneticEncoderFeedback(
        override val maxAngle: Double = this.maxAngle
    ) : IMotorFeedback {
        override val angularVelocity: Double
            get() = TODO("Not yet implemented")

        override val currentRawSteps: Long
            get() = TODO("Not yet implemented")

        override val currentAngle: Double
            get() = TODO("Not yet implemented")

        override val isAtLimitSwitch: Boolean
            get() = TODO("Not yet implemented")
    }

}