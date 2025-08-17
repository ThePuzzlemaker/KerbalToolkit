package com.teamisotope.kerbtk

import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
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
import kotlinx.coroutines.launch
import kotlin.time.Duration
import org.jetbrains.compose.ui.tooling.preview.Preview
import org.koin.compose.KoinApplication

@Composable
@Preview
fun App() {
  KoinApplication(application = {

  }) {
    KerbTkTheme {
      val appCoroutineScope = rememberCoroutineScope()
      val krpcClient by remember { mutableStateOf(null) }
      FloatingWindowCanvas {
        val menuState = remember { WindowState() }
        val timeUtilsState = remember { WindowState() }
        FloatingWindow(
          title = { Text("Hello, world!") },
          id = "menu",
          modifier = Modifier.height(256.dp).heightIn(256.dp, 512.dp).width(256.dp),
          collapsible = true,
          closable = false,
          state = menuState,
        ) {
          Text(
            "Hello, world! This is a test of blah blah blah words!!!!",
            modifier = Modifier.clickable { timeUtilsState.open = true },
          )
          KtkButton(onClick = {}) { Text("Connect to kRPC (TODO: proper window)") }
        }

        FloatingWindow(
          title = { Text("Time Utilities") },
          id = "timeutils",
          modifier = Modifier.width(384.dp),
          collapsible = true,
          closable = true,
          state = timeUtilsState,
        ) {
          Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text("Reference Vessel: TODO")
            KtkHorizontalSeparator()
            var t1 by remember { mutableStateOf(Duration.ZERO) }
            var t2 by remember { mutableStateOf(Duration.ZERO) }
            var adding by remember { mutableStateOf(true) }
            val t3 = if (adding) t1 + t2 else t1 - t2
            TimeInput(value = t1, onChanged = { t1 = it }, allowNegative = true)
            Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
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
    }
  }
}
