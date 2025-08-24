package com.teamisotope.kerbtk.displays

object AllDisplays {
  val registry =
    Registry<Display>("ktk.display") { id, display ->
      val res = this.all().find { d -> d.value.number == display.number }
      if (res != null)
        throw IllegalArgumentException(
          "Display number ${display.number} from `$id` conflicts with `${res.id}`"
        )
    }

  val timeUtils: RegistryObject<TimeUtils> = registry.register("ktk.timeUtils", ::TimeUtils)
}
