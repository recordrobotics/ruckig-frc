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

val osName = System.getProperty("os.name").lowercase()
val nativeLibDir = when {
    osName.contains("win") -> "windows"
    osName.contains("mac") -> "macos"
    osName.contains("nix") || osName.contains("nux") -> "linux"
    else -> "unknown"
}

sourceSets {
    main {
        java.srcDir("org/recordrobotics/ruckig")
        resources.srcDir("./build/jni")
        java.exclude("org/recordrobotics/ruckig/test/**")
    }
}

tasks.register<Copy>("copyNativeLib") {
    from("./build/jni/$nativeLibDir")
    into("build/resources/main/$nativeLibDir")
}

tasks.named<Jar>("jar") {
    dependsOn("copyNativeLib")
    duplicatesStrategy = DuplicatesStrategy.EXCLUDE
    from("build/resources/main/$nativeLibDir") {
        into(nativeLibDir)
    }
    exclude("org/recordrobotics/ruckig/test/**")
}

tasks.named<Jar>("sourcesJar") {
    exclude("org/recordrobotics/ruckig/test/**")
}

tasks.named<Javadoc>("javadoc") {
    dependsOn("copyNativeLib")
    exclude("org/recordrobotics/ruckig/test/**")
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
    }

    repositories {
        maven {
            url = layout.buildDirectory.dir("deploy").get().asFile.toURI()
        }
    }
}
