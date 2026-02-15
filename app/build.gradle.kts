plugins {
    // Apply the shared build logic from a convention plugin.
    // The shared code is located in `buildSrc/src/main/kotlin/kotlin-jvm.gradle.kts`.
    id("buildsrc.convention.kotlin-jvm")

    // Apply the Application plugin to add support for building an executable JVM application.
    application
}

dependencies {
    // Project "app" depends on project "utils". (Project paths are separated with ":", so ":utils" refers to the top-level "utils" project.)
    implementation(project(":utils"))

    implementation(libs.pi4j.core)
    implementation(libs.pi4j.ktx)
    implementation(libs.pi4j.plugin.gpiod)
    implementation(libs.pi4j.library.gpiod)
    implementation (libs.kotlinxCoroutines)
}

tasks.withType<Jar> {
    // This allows you to run 'java -jar app.jar'
    manifest {
        attributes["Main-Class"] = "dev.sebastianb.ballcatcher.app.AppKt"
    }

    // This bundles all dependencies into the JAR
    duplicatesStrategy = DuplicatesStrategy.EXCLUDE
    from(configurations.runtimeClasspath.get().map { if (it.isDirectory) it else zipTree(it) })
}

application {
    // Define the Fully Qualified Name for the application main class
    // (Note that Kotlin compiles `App.kt` to a class with FQN `com.example.app.AppKt`.)
    mainClass = "dev.sebastianb.ballcatcher.app.AppKt"
}
