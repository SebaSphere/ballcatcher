package dev.sebastianb.ballcatcher.app.api.dto

import kotlinx.serialization.Serializable

@Serializable
data class MoveToAngleRequest(val degrees: Float)

@Serializable
data class MoveToPositionRequest(val fraction: Float)

@Serializable
data class MoveClockwiseRequest(val degrees: Float)

@Serializable
data class MoveCounterClockwiseRequest(val degrees: Float)

@Serializable
data class SetEnabledRequest(val enabled: Boolean)

@Serializable
data class CommandResponse(val success: Boolean, val message: String)
