plugins {
    `java-library`
    `maven-publish`
}

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(17))
    }
    withJavadocJar()
    withSourcesJar()
}

group = "org.recordrobotics.ruckig"
version = System.getenv("GITHUB_REF_NAME") ?: "0.1.0-SNAPSHOT"

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

publishing {
    publications {
        create<MavenPublication>("gpr") {
            from(components["java"])
            groupId = project.group.toString()
            artifactId = "ruckig-frc"
            version = project.version.toString()
        }
    }
    repositories {
        maven {
            name = "GitHubPackages"
            url = uri("https://maven.pkg.github.com/${System.getenv("GITHUB_REPOSITORY")}")
            credentials {
                username = System.getenv("GITHUB_ACTOR")
                password = System.getenv("GITHUB_TOKEN")
            }
        }
    }
}
