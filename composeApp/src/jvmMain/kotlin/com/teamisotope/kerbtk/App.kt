package com.teamisotope.kerbtk

import androidx.compose.foundation.layout.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import com.composables.core.HorizontalSeparator
import com.composeunstyled.Text
import com.teamisotope.kerbtk.widgets.*
import org.jetbrains.compose.ui.tooling.preview.Preview
import kotlin.time.Duration

@Composable
@Preview
fun App() {
  KerbTkTheme {
    WindowCanvas {
      val menuState = remember { WindowState() }
      Window(
        title = { Text("Hello, world!") },
        id = "menu",
        modifier = Modifier.height(256.dp).heightIn(256.dp, 512.dp).width(256.dp),
        collapsible = true,
        state = menuState,
      ) {
        Text("Hello, world! This is a test of blah blah blah words!!!!")
      }

      val timeUtilsState = remember { WindowState() }
      Window(
        title = { Text("Time Utilities") },
        id = "timeutils",
        modifier = Modifier.width(384.dp),
        collapsible = true,
        state = timeUtilsState,
      ) {
        Column {
          Text("Reference Vessel: TODO")
          HorizontalSeparator(mauveDark[6], modifier = Modifier.padding(top = 8.dp, bottom = 8.dp))
          var tm by remember { mutableStateOf(Duration.ZERO) }
          TimeInput(
            value = tm,
            onChanged = { tm = it },
          )
        }
      }
    }
  }
}
