package com.teamisotope.kerbtk.displays

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.width
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.adamglin.PhosphorIcons
import com.adamglin.phosphoricons.Regular
import com.adamglin.phosphoricons.regular.ArrowsDownUp
import com.adamglin.phosphoricons.regular.Minus
import com.adamglin.phosphoricons.regular.Plus
import com.composeunstyled.Text
import com.teamisotope.kerbtk.widgets.*
import kotlin.time.Duration

class TimeUtils : Display(category = AllCategories.utils, number = 100, name = "Time Utilities") {
  private var state = WindowState()

  @Composable
  override fun build(scope: WindowCanvasScope) {
    scope.FloatingWindow(
      title = { Text("Time Utilities") },
      id = "timeutils",
      modifier = Modifier.width(384.dp),
      collapsible = true,
      closable = true,
      state = state,
    ) {
      Column(verticalArrangement = Arrangement.spacedBy(Theme.standardSpacing)) {
        Text("Reference Vessel: TODO")
        KtkHorizontalSeparator()
        var t1 by remember { mutableStateOf(Duration.ZERO) }
        var t2 by remember { mutableStateOf(Duration.ZERO) }
        var adding by remember { mutableStateOf(true) }
        val t3 = if (adding) t1 + t2 else t1 - t2
        TimeInput(value = t1, onChanged = { t1 = it }, allowNegative = true)
        Row(horizontalArrangement = Arrangement.spacedBy(Theme.standardSpacing)) {
          KtkButton(onClick = { t1 = t2.also { t2 = t1 } }) {
            KtkIcon(PhosphorIcons.Regular.ArrowsDownUp)
          }
          KtkSelectButton(selected = adding, onClick = { adding = true }) {
            KtkIcon(PhosphorIcons.Regular.Plus)
          }
          KtkSelectButton(selected = !adding, onClick = { adding = false }) {
            KtkIcon(PhosphorIcons.Regular.Minus)
          }
        }
        TimeInput(value = t2, onChanged = { t2 = it }, allowNegative = true)
        KtkHorizontalSeparator()
        TimeInput(value = t3, onChanged = {}, allowNegative = true, interactive = false)
      }
    }
  }

  override var isVisible: Boolean
    get() = state.open
    set(it) {
      state.open = it
    }
}
