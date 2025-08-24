plugins { alias(libs.plugins.kotlinMultiplatform) }

group = "com.teamisotope.gravitas"

version = "0.1.0"

kotlin {
  jvm()

  sourceSets {
    commonMain.dependencies {}

    jvmTest.dependencies { implementation(libs.kotlin.test) }
    jvmMain.dependencies {}
  }
}
