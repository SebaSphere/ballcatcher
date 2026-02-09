package dev.sebastianb.ballcatcher.app

import com.pi4j.Pi4J
import com.pi4j.io.gpio.digital.PullResistance
import com.pi4j.ktx.io.digital.digitalInput
import com.pi4j.ktx.io.digital.onLow
import com.pi4j.library.pigpio.PiGpio
import com.pi4j.plugin.pigpio.provider.gpio.digital.PiGpioDigitalInputProvider
import java.sql.Time
import java.time.Instant

fun main() {
    var magnetHit = 0

    val piGpio = PiGpio.newNativeInstance()

    val pi4j = Pi4J.newContextBuilder()
        .add(PiGpioDigitalInputProvider.newInstance(piGpio))
        .build()

    val sensor = pi4j.digitalInput(23) {
        id("magnet-sensor")
        pull(PullResistance.PULL_UP)
    }

    sensor.onLow {
        magnetHit++
        println("Magnet has been activated $magnetHit times at ${Time.from(Instant.now())}")
    }
    readln()
}