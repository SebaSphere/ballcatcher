plugins {
    id("buildsrc.convention.kotlin-jvm")
    application
}

dependencies {
    implementation(libs.opencv)
    testImplementation(kotlin("test"))
}

application {
    mainClass = "dev.sebastianb.ballcatcher.opencv.TriangulateAppKt"
}
