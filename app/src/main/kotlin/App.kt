package dev.sebastianb.ballcatcher.app

import com.pi4j.Pi4J
import com.pi4j.context.Context
import com.pi4j.io.gpio.digital.DigitalOutput
import com.pi4j.io.gpio.digital.DigitalState
import com.pi4j.io.gpio.digital.PullResistance
import com.pi4j.ktx.io.digital.digitalInput
import com.pi4j.ktx.io.digital.onLow
import java.sql.Time
import java.time.Instant
import kotlinx.coroutines.*
import kotlin.math.PI
import kotlin.math.cos

// Pi4j uses BCM
// This correlates to the GPIO number at https://cdn.shopify.com/s/files/1/0195/1344/2404/files/pi-5-diagram.jpg?v=1762784407 on a RP5

const val MAGNET_PIN = 23
const val PULSE_PIN = 20
const val DIRECTION_PIN = 21

val CW_DIRECTION = DigitalState.LOW
val CCW_DIRECTION = DigitalState.HIGH


suspend fun main() {

    val pi4j = Pi4J.newAutoContext()

    coroutineScope {
        launch {
            readMagnet(pi4j)
        }
        launch {
            controlStepperTest(pi4j)
        }
    }

    awaitCancellation()

}

suspend fun readMagnet(pi4j: Context) {
    var magnetHit = 0

    val sensor = pi4j.digitalInput(MAGNET_PIN) {
        id("magnet-sensor")
        address(23)
        pull(PullResistance.PULL_UP)
        debounce(5000L) // 5ms debounce to prevent "chatter"
    }

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
        println("Starting Cosine Wave Test...")
        direction.state(CW_DIRECTION)

        // Run for 400 steps to see a full speed-up/slow-down cycle
        repeat(400) { i ->
            // Calculate a value from 0 to 1 based on a cosine wave
            // We use (i / 400.0) to normalize the loop over 2*PI
            val cosValue = cos(2.0 * PI * (i / 400.0))

            // Map cosValue (-1 to 1) to a delay range (2ms to 20ms)
            // (cosValue + 1) / 2 gives us a range of 0 to 1
            val variableDelay = (2 + ((cosValue + 1) / 2) * 18).toLong()

            pulse.high()
            delay(variableDelay)
            pulse.low()
            delay(variableDelay)
        }

//        println("Direction CCW")
//        delay(500)
//        direction.state(CCW_DIRECTION)
//
//        repeat(200) {
//            pulse.high()
//            delay(1)
//            pulse.low()
//            delay(1)
//        }
    }

}