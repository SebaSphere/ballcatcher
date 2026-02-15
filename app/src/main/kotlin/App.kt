package dev.sebastianb.ballcatcher.app

import com.pi4j.Pi4J
import com.pi4j.context.Context
import com.pi4j.io.gpio.digital.PullResistance
import com.pi4j.ktx.io.digital.digitalInput
import com.pi4j.ktx.io.digital.onLow
import com.pi4j.plugin.gpiod.provider.gpio.digital.GpioDDigitalInputProvider
import java.sql.Time
import java.time.Instant
import kotlinx.coroutines.*


suspend fun main() {


    val pi4j = Pi4J.newContextBuilder()
        .add(GpioDDigitalInputProvider.newInstance())
        .setGpioChipName("gpiochip0")
        .build()


    withContext(Dispatchers.Default) {
        this.launch {
            readMagnet(pi4j)
        }
        this.launch {
            controlStepperTest()
        }
    }

    readln()
    pi4j.shutdown()
}

suspend fun readMagnet(pi4j: Context) {
    var magnetHit = 0

    // BCM 23 is Physical Pin 16
    val sensor = pi4j.digitalInput(23) {
        id("magnet-sensor")
        address(23)
        pull(PullResistance.PULL_UP)
        debounce(5000L) // 5ms debounce to prevent "chatter"
    }

    sensor.onLow {
        magnetHit++
        println("Magnet has been hurt $magnetHit times at ${Time.from(Instant.now())}. Magnet needs therapy ):")
    }
    readln() // Keep the process alive
}

suspend fun controlStepperTest() {
    println("Control StepperTest")

}