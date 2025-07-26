import java.io.ByteArrayOutputStream

plugins {
    `java-library`
    `maven-publish`
    id("org.jreleaser") version "1.19.0"
}

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(17))
    }
    withJavadocJar()
    withSourcesJar()
}

group = "org.recordrobotics.ruckig"
version = (System.getenv("GITHUB_REF_NAME")?.removePrefix("v")) ?: run {
    val stdout = ByteArrayOutputStream()
    project.exec {
        commandLine = listOf("git", "describe", "--tags", "--abbrev=0")
        standardOutput = stdout
        isIgnoreExitValue = true
    }
    val tag = stdout.toString().trim().removePrefix("v").ifEmpty { "0.1.0" }
    "$tag-SNAPSHOT"
}

sourceSets {
    main {
        java.srcDir("org/recordrobotics/ruckig")
        resources.srcDir("./build/jni")
        java.exclude("org/recordrobotics/ruckig/test/**")
    }
}

tasks.named<Jar>("jar") {
    exclude("org/recordrobotics/ruckig/test/**")
}

tasks.named<Jar>("sourcesJar") {
    exclude("org/recordrobotics/ruckig/test/**")
}

tasks.named<Javadoc>("javadoc") {
    exclude("org/recordrobotics/ruckig/test/**")
}

// Create tasks to zip native libraries for each platform
val nativePlatforms = mapOf(
    "windowsx86-64" to "windows/x86-64/shared",
    "osxuniversal" to "osx/universal/shared",
    "linuxx86-64" to "linux/x86-64/shared",
    "linuxathena" to "linux/athena/shared",
    "linuxarm64" to "linux/arm64/shared"
)

val nativeZips = nativePlatforms.map { (zipName, nativePath) ->
    tasks.register<Zip>("zipNative${zipName.replaceFirstChar { it.uppercase() }}") {
        group = "build"
        archiveBaseName.set(zipName)
        destinationDirectory.set(layout.buildDirectory.dir("native-zips"))
        from(layout.buildDirectory.dir("jni/$nativePath"))
        into(nativePath)
    }
}

jreleaser {
    project {
        gitRootSearch.set(true)
    }
    signing {
        active.set(org.jreleaser.model.Active.ALWAYS)
        armored.set(true)
    }
    deploy {
        maven {
            mavenCentral {
                register("release-deploy") {
                    active.set(org.jreleaser.model.Active.RELEASE)
                    url.set("https://central.sonatype.com/api/v1/publisher")
                    stagingRepository(layout.buildDirectory.dir("deploy").get().asFile.absolutePath)
                }
            }
            nexus2 {
                register("snapshot-deploy") {
                    active.set(org.jreleaser.model.Active.SNAPSHOT)
                    snapshotUrl.set("https://central.sonatype.com/repository/maven-snapshots/")
                    applyMavenCentralRules.set(true)
                    snapshotSupported.set(true)
                    closeRepository.set(true)
                    releaseRepository.set(true)
                    stagingRepository(layout.buildDirectory.dir("deploy").get().asFile.absolutePath)
                }
            }
        }
    }
}

publishing {
    publications {
        create<MavenPublication>("gpr") {
            from(components["java"])
            groupId = project.group.toString()
            artifactId = "ruckig-frc"
            version = project.version.toString()

            pom {
                name.set("ruckig-frc")
                description.set("JNI bindings for Ruckig for FRC")
                url.set("https://github.com/recordrobotics/ruckig-frc")
                inceptionYear.set("2025")
                licenses {
                    license {
                        name.set("MIT License")
                        url.set("https://opensource.org/licenses/MIT")
                    }
                }
                developers {
                    developer {
                        id.set("recordrobotics")
                        name.set("Record Robotics")
                    }
                }
                scm {
                    connection.set("scm:git:https://github.com/recordrobotics/ruckig-frc.git")
                    developerConnection.set("scm:git:ssh://github.com:recordrobotics/ruckig-frc.git")
                    url.set("https://github.com/recordrobotics/ruckig-frc")
                }
            }
        }
        create<MavenPublication>("native") {
            groupId = project.group.toString()
            artifactId = "ruckig-native"
            version = project.version.toString()
            nativeZips.forEach { zipTask ->
                artifact(zipTask.get().archiveFile) {
                    classifier = zipTask.get().archiveBaseName.get()
                    extension = "zip"
                }
            }

            pom {
                name.set("ruckig-native")
                description.set("Native WPILib JNI bindings for Ruckig for FRC")
                url.set("https://github.com/recordrobotics/ruckig-frc")
                inceptionYear.set("2025")
                licenses {
                    license {
                        name.set("MIT License")
                        url.set("https://opensource.org/licenses/MIT")
                    }
                }
                developers {
                    developer {
                        id.set("recordrobotics")
                        name.set("Record Robotics")
                    }
                }
                scm {
                    connection.set("scm:git:https://github.com/recordrobotics/ruckig-frc.git")
                    developerConnection.set("scm:git:ssh://github.com:recordrobotics/ruckig-frc.git")
                    url.set("https://github.com/recordrobotics/ruckig-frc")
                }
            }
        }
    }

    repositories {
        maven {
            url = layout.buildDirectory.dir("deploy").get().asFile.toURI()
        }
    }
}
