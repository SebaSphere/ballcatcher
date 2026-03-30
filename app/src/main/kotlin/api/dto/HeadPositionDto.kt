package dev.sebastianb.ballcatcher.app.api.dto

import dev.sebastianb.ballcatcher.app.ship.HeadRotationalPosition
import kotlinx.serialization.Serializable

@Serializable
data class HeadPositionDto(
    val yaw: Float,
    val pitch: Float,
) {
    fun toModel() = HeadRotationalPosition(yaw = yaw, pitch = pitch)

    companion object {
        fun from(position: HeadRotationalPosition) = HeadPositionDto(
            yaw = position.yaw,
            pitch = position.pitch,
        )
    }
}
