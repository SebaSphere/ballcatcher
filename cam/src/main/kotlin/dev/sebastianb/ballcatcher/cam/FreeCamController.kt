package dev.sebastianb.ballcatcher.cam

import com.badlogic.gdx.Gdx
import com.badlogic.gdx.Input
import com.badlogic.gdx.InputAdapter
import com.badlogic.gdx.graphics.PerspectiveCamera
import com.badlogic.gdx.math.Vector3

class FreeCamController(
    private val camera: PerspectiveCamera,
    private val moveSpeed: Float = 10f,
    private val yawSpeed: Float = 90f
) : InputAdapter() {

    private val target = Vector3(0f, 0f, 0f) // orbit center (the cube)
    private val tmp = Vector3()

    fun update(deltaTime: Float) {
        // Q/E orbit around target
        if (Gdx.input.isKeyPressed(Input.Keys.Q)) {
            val angle = yawSpeed * deltaTime
            camera.position.sub(target)
            camera.position.rotate(Vector3.Y, angle)
            camera.position.add(target)
        }
        if (Gdx.input.isKeyPressed(Input.Keys.E)) {
            val angle = -yawSpeed * deltaTime
            camera.position.sub(target)
            camera.position.rotate(Vector3.Y, angle)
            camera.position.add(target)
        }

        // Always look at target
        camera.lookAt(target)
        camera.up.set(Vector3.Y)

        // WASD movement (relative to current camera orientation)
        val speed = moveSpeed
        val forward = Vector3(target).sub(camera.position).nor()
        val right = Vector3(forward).crs(Vector3.Y).nor()

        if (Gdx.input.isKeyPressed(Input.Keys.W)) {
            tmp.set(forward).scl(speed * deltaTime)
            camera.position.add(tmp)
            target.add(tmp)
        }
        if (Gdx.input.isKeyPressed(Input.Keys.S)) {
            tmp.set(forward).scl(-speed * deltaTime)
            camera.position.add(tmp)
            target.add(tmp)
        }
        if (Gdx.input.isKeyPressed(Input.Keys.A)) {
            tmp.set(right).scl(-speed * deltaTime)
            camera.position.add(tmp)
            target.add(tmp)
        }
        if (Gdx.input.isKeyPressed(Input.Keys.D)) {
            tmp.set(right).scl(speed * deltaTime)
            camera.position.add(tmp)
            target.add(tmp)
        }
        if (Gdx.input.isKeyPressed(Input.Keys.SPACE)) {
            camera.position.y += speed * deltaTime
            target.y += speed * deltaTime
        }
        if (Gdx.input.isKeyPressed(Input.Keys.SHIFT_LEFT)) {
            camera.position.y -= speed * deltaTime
            target.y -= speed * deltaTime
        }
    }

}
