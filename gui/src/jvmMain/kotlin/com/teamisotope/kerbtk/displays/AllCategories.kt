package com.teamisotope.kerbtk.displays

object AllCategories {
  val registry = Registry<DisplayCategory>("ktk.displayCategory") { _, _ -> }

  val config: RegistryObject<DisplayCategory> =
    registry.register("ktk.config") { DisplayCategory(name = "Configuration", "CFG") }
  val utils: RegistryObject<DisplayCategory> =
    registry.register("ktk.util") { DisplayCategory(name = "Utilities", "UTIL") }
}
