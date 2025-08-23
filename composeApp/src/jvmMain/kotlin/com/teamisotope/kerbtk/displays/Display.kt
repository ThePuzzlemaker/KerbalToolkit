package com.teamisotope.kerbtk.displays

import androidx.compose.runtime.Composable
import com.teamisotope.kerbtk.widgets.WindowCanvasScope

abstract class Display(
  val category: RegistryObject<DisplayCategory>,
  val number: Int,
  val name: String,
) {
  @Composable abstract fun build(scope: WindowCanvasScope)

  abstract var isVisible: Boolean
}

data class DisplayCategory(val name: String, val prefixes: List<String>) {
  constructor(name: String, vararg prefixes: String) : this(name, prefixes.toList()) {}
}

class Registry<T>(private val id: String, private val validator: Registry<T>.(String, T) -> Unit) {
  private val map = mutableMapOf<String, T>()

  fun <U : T> register(id: String, makeValue: () -> U): RegistryObject<U> {
    if (map.containsKey(id))
      throw IllegalArgumentException(
        "Item with ID `$id` was already present in registry `${this.id}`"
      )
    val value = makeValue()
    validator(id, value)
    map[id] = value
    return RegistryObject(id, value)
  }

  fun get(id: String): T? = map[id]

  fun all(): Iterable<RegistryObject<T>> = map.asIterable().map { RegistryObject(it.key, it.value) }
}

data class RegistryObject<T>(val id: String, val value: T) {}
