package dev.sebastianb.ballcatcher.app

import com.pi4j.Pi4J
import com.pi4j.context.Context
import com.pi4j.io.gpio.digital.DigitalOutput
import com.pi4j.io.gpio.digital.DigitalState
import com.pi4j.io.gpio.digital.PullResistance
import com.pi4j.ktx.io.digital.digitalInput
import com.pi4j.ktx.io.digital.onHigh
import com.pi4j.ktx.io.digital.onLow
import com.pi4j.plugin.gpiod.provider.gpio.digital.GpioDDigitalInputProvider
import com.pi4j.plugin.gpiod.provider.gpio.digital.GpioDDigitalOutputProvider
import dev.sebastianb.ballcatcher.app.motor.MotorState
import dev.sebastianb.ballcatcher.app.ship.YawJointController
import java.sql.Time
import java.time.Instant
import kotlinx.coroutines.*

suspend fun main() {


    val pi4j = Pi4J.newContextBuilder()
        .add(GpioDDigitalInputProvider.newInstance())  // For the Magnet
        .add(GpioDDigitalOutputProvider.newInstance()) // For the Stepper Pulse/Dir
        .build()

    coroutineScope {
        val yawController = YawJointController(pi4j)
        yawController.motorControl.setEnabled(true)
        yawController.motorControl.calibrateHome()

        launch {
            while (isActive) {
                yawController.update()
                yield()
            }
        }

        launch {
            while (isActive) {
                println("Current Angle: ${yawController.motorFeedback.currentAngle}, Magnet On: ${yawController.motorFeedback.isOn}")
                delay(1000)
            }
        }

        launch {
            repeat(100) { iteration ->
                println("Starting CW/CCW iteration ${iteration + 1}")
                
                println("Moving 90 degrees Clockwise")
                yawController.motorControl.moveClockwise(90f)
                delay(1000)

                println("Moving 45 degrees Counter-Clockwise")
                yawController.motorControl.moveCounterClockwise(45f)
                delay(1000)

                println("Moving 45 degrees Counter-Clockwise")
                yawController.motorControl.moveCounterClockwise(45f)
                delay(1000)

                println("Moving 180 degrees Clockwise")
                yawController.motorControl.moveClockwise(180f)
                delay(1000)

                println("Moving 360 degrees Counter-Clockwise")
                yawController.motorControl.moveCounterClockwise(360f)
                delay(1000)

                println("Moving back to 0")
                yawController.motorControl.moveToAngle(0f)
                delay(2000)
            }
            println("Movement sequence complete.")
        }
    }

    awaitCancellation()

}

// These are now replaced by YawJointController
/*
suspend fun readMagnet(pi4j: Context) {
    var magnetHit = 0

    val sensor = pi4j.digitalInput(MAGNET_PIN) {
        id("magnet-sensor")
        address(MAGNET_PIN)
        pull(PullResistance.PULL_UP)
        debounce(5000L) // 5ms debounce to prevent "chatter"
    }
    println("Magnet sensor initialized. Current state: ${sensor.state()}")

    sensor.onLow {
        magnetHit++
        println("Magnet has been hurt $magnetHit times at ${Time.from(Instant.now())}. Magnet needs therapy ):")
    }
}

suspend fun controlStepperTest(pi4j: Context) {
    println("Control StepperTest")

    val configDir = DigitalOutput.newConfigBuilder(pi4j)
        .address(DIRECTION_PIN)
        .id("direction")
        .build()
    val direction = pi4j.create(configDir)

    val configPulse = DigitalOutput.newConfigBuilder(pi4j)
        .address(PULSE_PIN)
        .id("pulse")
        .build()
    val pulse = pi4j.create(configPulse)

    while (true) {
        delay(500)
        direction.state(CW_DIRECTION)

        val totalSteps = 400

        var delayMs = 2L
        val stopRampStart = totalSteps - 50 // Start slowing down 50 steps before the end

        // from what I understand, it's supposed to pulse steps (thus the high and low)
        repeat(totalSteps) { i ->
            pulse.high()
            delay(delayMs)
            pulse.low()
            delay(delayMs)

            // Deceleration Logic: Gradually increase delay to slow down
            if (i > stopRampStart && delayMs < 5) {
                if (i % 10 == 0) delayMs++
            }
        }

        delay(500)
        direction.state(CCW_DIRECTION)

        delayMs = 2L

        repeat(totalSteps) { i ->
            pulse.high()
            delay(delayMs)
            pulse.low()
            delay(delayMs)

            // Deceleration Logic: Gradually increase delay to slow down
            if (i > stopRampStart && delayMs < 5) {
                if (i % 10 == 0) delayMs++
            }
        }
    }
}
*/