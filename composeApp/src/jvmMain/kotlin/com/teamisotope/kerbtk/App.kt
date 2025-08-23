package com.teamisotope.kerbtk

import androidx.compose.foundation.layout.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.adamglin.PhosphorIcons
import com.adamglin.phosphoricons.Fill
import com.adamglin.phosphoricons.fill.CheckFat
import com.composeunstyled.Text
import com.teamisotope.kerbtk.displays.AllCategories
import com.teamisotope.kerbtk.displays.AllDisplays
import com.teamisotope.kerbtk.krpc.KrpcClient
import com.teamisotope.kerbtk.widgets.*
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import org.jetbrains.compose.ui.tooling.preview.Preview

@Composable
@Preview
fun App() {
  KerbTkRoot {
    val appCoroutineScope = rememberCoroutineScope()
    var krpcClient by remember { mutableStateOf<KrpcClient?>(null) }
    val openDisplays = remember { mutableStateMapOf<String, Boolean>() }
    FloatingWindowCanvas {
      val menuState = remember {
        val x = WindowState()
        x.open = true
        x
      }
      val timeUtilsState = remember { WindowState() }
      FloatingWindow(
        title = { Text("Menu") },
        id = "menu",
        modifier = Modifier.height(256.dp).heightIn(256.dp, 512.dp).width(256.dp),
        collapsible = true,
        closable = false,
        state = menuState,
      ) {
        Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
          for (category in AllCategories.registry.all().sortedBy { it.value.name }) {
            key(category.id) { Text("${category.value.prefixes} / ${category.value.name}") }
            for (display in
              AllDisplays.registry.all().filter { it.value.category.id == category.id }) {
              key(display.id) {
                KtkCheckbox(
                  icon = { KtkIcon(PhosphorIcons.Fill.CheckFat) },
                  checked = display.value.isVisible,
                  onCheckedChange = { display.value.isVisible = it },
                ) {
                  Text(
                    "${category.value.prefixes[0]}${display.value.number}: ${display.value.name}"
                  )
                }
              }
            }
          }
          KtkButton(
            onClick = {
              appCoroutineScope.launch(Dispatchers.IO) {
                try {
                  krpcClient = KrpcClient.connect("kerbtk", "127.0.0.1", 50000)
                } catch (e: Exception) {
                  e.printStackTrace()
                }
              }
            }
          ) {
            Text("Connect to kRPC (TODO: proper window)")
          }
        }
      }

      for (display in AllDisplays.registry.all()) {
        key(display.id) { display.value.build(this) }
      }
    }
  }
}
