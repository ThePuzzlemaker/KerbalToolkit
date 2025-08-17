import org.jetbrains.compose.desktop.application.dsl.TargetFormat

plugins {
  alias(libs.plugins.kotlinMultiplatform)
  alias(libs.plugins.composeMultiplatform)
  alias(libs.plugins.composeCompiler)
  alias(libs.plugins.composeHotReload)
  kotlin("plugin.serialization") version "2.2.10"
}

kotlin {
  jvm()

  sourceSets {
    commonMain.dependencies {
      implementation(compose.runtime)
      implementation(compose.foundation)
      implementation(compose.material3)
      implementation(compose.ui)
      implementation(compose.components.resources)
      implementation(compose.components.uiToolingPreview)
      implementation(libs.androidx.lifecycle.viewmodelCompose)
      implementation(libs.androidx.lifecycle.runtimeCompose)
    }
    commonTest.dependencies { implementation(libs.kotlin.test) }
    jvmMain.dependencies {
      implementation(compose.desktop.currentOs)
      implementation(libs.kotlinx.coroutinesSwing)
      implementation("com.adamglin:phosphor-icon:1.0.0")
      implementation("com.composables:core:1.40.0")
      implementation("io.ktor:ktor-server-netty:3.2.3")
      implementation("io.ktor:ktor-network:3.2.3")
      implementation("org.jetbrains.kotlinx:kotlinx-serialization-protobuf:1.9.0")
      implementation("io.insert-koin:koin-core:4.0.3")
      implementation("io.insert-koin:koin-compose:4.0.3")
      implementation("io.insert-koin:koin-compose-viewmodel:4.0.3")
      implementation("io.insert-koin:koin-compose-viewmodel-navigation:4.0.3")
    }
  }
}

compose.desktop {
  application {
    mainClass = "com.teamisotope.kerbtk.MainKt"

    nativeDistributions {
      targetFormats(TargetFormat.Dmg, TargetFormat.Msi, TargetFormat.Deb)
      packageName = "com.teamisotope.kerbtk"
      packageVersion = "1.0.0"
    }
  }
}
