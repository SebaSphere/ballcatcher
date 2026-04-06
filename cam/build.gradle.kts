plugins {
    id("buildsrc.convention.kotlin-jvm")
    application
}

dependencies {
    implementation(libs.libgdx.core)
    implementation(libs.libgdx.backend.lwjgl3)
    implementation(variantOf(libs.libgdx.platform.desktop) { classifier("natives-desktop") })
}

tasks.withType<Jar> {
    manifest {
        attributes["Main-Class"] = "dev.sebastianb.ballcatcher.cam.CamAppKt"
    }

    duplicatesStrategy = DuplicatesStrategy.EXCLUDE
    from(configurations.runtimeClasspath.get().map { if (it.isDirectory) it else zipTree(it) })
}

application {
    mainClass = "dev.sebastianb.ballcatcher.cam.CamAppKt"
}
