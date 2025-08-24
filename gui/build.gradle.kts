import org.jetbrains.compose.desktop.application.dsl.TargetFormat

plugins {
  alias(libs.plugins.kotlinMultiplatform)
  alias(libs.plugins.composeMultiplatform)
  alias(libs.plugins.composeCompiler)
  alias(libs.plugins.composeHotReload)
  alias(libs.plugins.kotlinSerialization)
}

kotlin {
  jvm()

  sourceSets {
    commonMain.dependencies {
      implementation(compose.runtime)
      implementation(compose.foundation)
      implementation(compose.ui)
      implementation(compose.components.resources)
      implementation(compose.components.uiToolingPreview)

      implementation(libs.androidx.lifecycle.viewmodelCompose)
      implementation(libs.androidx.lifecycle.runtimeCompose)
    }
    jvmTest.dependencies { implementation(libs.kotlin.test) }
    jvmMain.dependencies {
      implementation(compose.desktop.currentOs)
      implementation(libs.kotlinx.coroutinesSwing)

      implementation(libs.ktor.serverNetty)
      implementation(libs.ktor.network)

      implementation(libs.kotlinx.serializationProtobuf)

      implementation(libs.phosphorIcon)
      implementation(libs.composables.core)
    }
  }
}

compose.desktop {
  application {
    mainClass = "com.teamisotope.kerbtk.MainKt"

    nativeDistributions {
      targetFormats(TargetFormat.Dmg, TargetFormat.Exe, TargetFormat.AppImage)
      packageName = "com.teamisotope.kerbtk"
      packageVersion = "1.0.0"
    }
  }
}
