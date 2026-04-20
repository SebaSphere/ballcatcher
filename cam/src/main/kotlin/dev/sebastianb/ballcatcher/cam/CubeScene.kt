package dev.sebastianb.ballcatcher.cam

import com.badlogic.gdx.ApplicationAdapter
import com.badlogic.gdx.Gdx
import com.badlogic.gdx.Input
import com.badlogic.gdx.graphics.Color
import com.badlogic.gdx.graphics.GL20
import com.badlogic.gdx.graphics.PerspectiveCamera
import com.badlogic.gdx.graphics.VertexAttributes.Usage
import com.badlogic.gdx.graphics.g2d.BitmapFont
import com.badlogic.gdx.graphics.g2d.SpriteBatch
import com.badlogic.gdx.graphics.g3d.*
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder
import com.badlogic.gdx.math.Matrix4
import com.badlogic.gdx.math.Vector3
import com.badlogic.gdx.utils.ScreenUtils
import java.io.File
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import kotlin.random.Random

class CubeScene : ApplicationAdapter() {

    private lateinit var mainCamera: PerspectiveCamera
    private lateinit var leftCamera: PerspectiveCamera
    private lateinit var rightCamera: PerspectiveCamera
    private lateinit var modelBatch: ModelBatch
    private lateinit var sphereModel: Model
    private lateinit var sphereInstance: ModelInstance
    private lateinit var environment: Environment
    private lateinit var camController: FreeCamController
    private lateinit var spriteBatch: SpriteBatch
    private lateinit var font: BitmapFont
    // Half the interpupillary distance — each eye offsets by this amount
    private val eyeOffset = 0.5f

    override fun create() {
        val halfWidth = Gdx.graphics.width.toFloat() / 2f
        val height = Gdx.graphics.height.toFloat()

        mainCamera = PerspectiveCamera(67f, halfWidth, height).apply {
            position.set(5f, 0f, 5f)
            lookAt(0f, 0f, 0f)
            near = 0.1f
            far = 300f
            update()
        }

        leftCamera = PerspectiveCamera(67f, halfWidth, height).apply {
            near = 0.1f
            far = 300f
        }

        rightCamera = PerspectiveCamera(67f, halfWidth, height).apply {
            near = 0.1f
            far = 300f
        }

        environment = Environment().apply {
            set(ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f))
            add(DirectionalLight().set(Color.WHITE, -1f, -0.8f, -0.2f))
        }

        modelBatch = ModelBatch()

        val modelBuilder = ModelBuilder()
        sphereModel = modelBuilder.createSphere(
            2f, 2f, 2f, 20, 20,
            Material(ColorAttribute.createDiffuse(Color.GREEN)),
            (Usage.Position or Usage.Normal).toLong()
        )
        sphereInstance = ModelInstance(sphereModel).apply {
            transform.setTranslation(
                Random.nextFloat() * 20f - 10f,
                0f,
                Random.nextFloat() * 20f - 10f
            )
        }

        camController = FreeCamController(mainCamera)
        Gdx.input.inputProcessor = camController

        spriteBatch = SpriteBatch()
        font = BitmapFont() // default LibGDX font
        font.color = Color.WHITE
    }

    override fun resize(width: Int, height: Int) {
        val halfW = width.toFloat() / 2f
        val h = height.toFloat()
        mainCamera.viewportWidth = halfW
        mainCamera.viewportHeight = h
        mainCamera.update()
        leftCamera.viewportWidth = halfW
        leftCamera.viewportHeight = h
        rightCamera.viewportWidth = halfW
        rightCamera.viewportHeight = h
    }

    override fun render() {
        camController.update(Gdx.graphics.deltaTime)
        mainCamera.update()

        // Compute the right vector of the main camera
        val forward = Vector3(mainCamera.direction).nor()
        val right = Vector3(forward).crs(mainCamera.up).nor()

        // Left eye: offset to the left
        leftCamera.position.set(mainCamera.position).add(Vector3(right).scl(-eyeOffset))
        leftCamera.direction.set(mainCamera.direction)
        leftCamera.up.set(mainCamera.up)
        leftCamera.update()

        // Right eye: offset to the right
        rightCamera.position.set(mainCamera.position).add(Vector3(right).scl(eyeOffset))
        rightCamera.direction.set(mainCamera.direction)
        rightCamera.up.set(mainCamera.up)
        rightCamera.update()

        val w = Gdx.graphics.backBufferWidth
        val h = Gdx.graphics.backBufferHeight
        val halfW = w / 2

        // Clear the whole screen once
        Gdx.gl.glViewport(0, 0, w, h)
        Gdx.gl.glClearColor(0.1f, 0.1f, 0.1f, 1f)
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT or GL20.GL_DEPTH_BUFFER_BIT)

        // Render left eye (left half of screen)
        Gdx.gl.glViewport(0, 0, halfW, h)
        modelBatch.begin(leftCamera)
        modelBatch.render(sphereInstance, environment)
        modelBatch.end()

        // Render right eye (right half of screen)
        Gdx.gl.glViewport(halfW, 0, w - halfW, h)
        modelBatch.begin(rightCamera)
        modelBatch.render(sphereInstance, environment)
        modelBatch.end()

        // HUD overlay — reset viewport to full screen for 2D text
        Gdx.gl.glViewport(0, 0, w, h)
        spriteBatch.projectionMatrix = Matrix4().setToOrtho2D(0f, 0f, w.toFloat(), h.toFloat())
        spriteBatch.begin()

        val lp = leftCamera.position
        val rp = rightCamera.position
        val dist = lp.dst(rp)

        val padding = 10f
        val lineHeight = 18f
        val topY = h.toFloat() - padding

        // Left camera info (top-left of left viewport)
        font.draw(spriteBatch, "Left Cam", padding, topY)
        font.draw(spriteBatch, "X: %.2f".format(lp.x), padding, topY - lineHeight)
        font.draw(spriteBatch, "Y: %.2f".format(lp.y), padding, topY - lineHeight * 2)
        font.draw(spriteBatch, "Z: %.2f".format(lp.z), padding, topY - lineHeight * 3)

        // Right camera info (top-left of right viewport)
        val rightX = halfW.toFloat() + padding
        font.draw(spriteBatch, "Right Cam", rightX, topY)
        font.draw(spriteBatch, "X: %.2f".format(rp.x), rightX, topY - lineHeight)
        font.draw(spriteBatch, "Y: %.2f".format(rp.y), rightX, topY - lineHeight * 2)
        font.draw(spriteBatch, "Z: %.2f".format(rp.z), rightX, topY - lineHeight * 3)

        // Distance between cameras (below left camera info)
        font.draw(spriteBatch, "Dist: %.2f".format(dist), padding, topY - lineHeight * 5)

        spriteBatch.end()

        // Screenshot on C press
        if (Gdx.input.isKeyJustPressed(Input.Keys.C)) {
            takeStereoscreenshot(halfW, h)
        }
    }

    private fun takeStereoscreenshot(halfW: Int, h: Int) {
        val timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss-SSS"))
        val dir = File("screenshots/$timestamp")
        dir.mkdirs()

        // Capture left viewport (cam1)
        val leftPixmap = ScreenUtils.getFrameBufferPixmap(0, 0, halfW, h)
        // Capture right viewport (cam2)
        val rightPixmap = ScreenUtils.getFrameBufferPixmap(halfW, 0, halfW, h)

        val flippedLeft = flipVertically(leftPixmap)
        val flippedRight = flipVertically(rightPixmap)
        leftPixmap.dispose()
        rightPixmap.dispose()

        val leftFile = com.badlogic.gdx.files.FileHandle(File(dir, "cam1.png"))
        val rightFile = com.badlogic.gdx.files.FileHandle(File(dir, "cam2.png"))

        com.badlogic.gdx.graphics.PixmapIO.writePNG(leftFile, flippedLeft)
        com.badlogic.gdx.graphics.PixmapIO.writePNG(rightFile, flippedRight)

        flippedLeft.dispose()
        flippedRight.dispose()

        val lp = leftCamera.position
        val rp = rightCamera.position
        val dir3 = mainCamera.direction
        val up3 = mainCamera.up
        File(dir, "cameras.txt").writeText(
            "left_cam %.4f %.4f %.4f\nright_cam %.4f %.4f %.4f\ndirection %.4f %.4f %.4f\nup %.4f %.4f %.4f\n".format(
                lp.x, lp.y, lp.z, rp.x, rp.y, rp.z,
                dir3.x, dir3.y, dir3.z, up3.x, up3.y, up3.z
            )
        )

        val sp = Vector3()
        sphereInstance.transform.getTranslation(sp)
        File(dir, "sphere.txt").writeText(
            "sphere %.4f %.4f %.4f\n".format(sp.x, sp.y, sp.z)
        )

        println("Screenshots saved to screenshots/$timestamp/")
    }

    private fun flipVertically(src: com.badlogic.gdx.graphics.Pixmap): com.badlogic.gdx.graphics.Pixmap {
        val w = src.width
        val h = src.height
        val flipped = com.badlogic.gdx.graphics.Pixmap(w, h, src.format)
        for (y in 0 until h) {
            flipped.drawPixmap(src, 0, y, w, 1, 0, h - 1 - y, w, 1)
        }
        return flipped
    }

    override fun dispose() {
        modelBatch.dispose()
        sphereModel.dispose()
        spriteBatch.dispose()
        font.dispose()
    }
}
