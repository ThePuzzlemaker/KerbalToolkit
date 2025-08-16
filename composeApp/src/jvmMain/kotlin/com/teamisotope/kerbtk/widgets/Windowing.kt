package com.teamisotope.kerbtk.widgets

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.*
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.zIndex
import com.composeunstyled.LocalTextStyle

private val ForegroundNode =
  compositionLocalOf<MutableState<String?>> { error("No window canvas present") }

@Composable
fun WindowCanvas(content: @Composable () -> Unit) {
  val fgNode = remember { mutableStateOf<String?>(null) }
  Box(Modifier.fillMaxSize().background(mauveDark[0])) {
    CompositionLocalProvider(ForegroundNode provides fgNode) { content() }
  }
}

class WindowState {
  var collapsed: Boolean by mutableStateOf(false)
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun Window(
  title: @Composable () -> Unit,
  /// A unique identifier (within the WindowCanvas) of this window.
  id: String,
  modifier: Modifier = Modifier,
  state: WindowState = WindowState(),
  collapsible: Boolean = true,
  content: @Composable () -> Unit,
) {
  var position by remember { mutableStateOf(Offset.Zero) }
  var fgNode by ForegroundNode.current
  // Box here allows us to constrain the size of the window without breaking collapsing.
  Box(modifier) {
    Column(
      Modifier.graphicsLayer(translationX = position.x, translationY = position.y)
        .zIndex(
          if (fgNode == id) {
            1f
          } else {
            0f
          }
        )
        .background(color = mauveDark[2], shape = RoundedCornerShape(size = 8.dp))
        .border(width = 1.dp, color = mauveDark[5], shape = RoundedCornerShape(size = 8.dp))
        .onClick { fgNode = id }
    ) {
      Row(
        Modifier.height(height = 32.dp)
          .fillMaxWidth()
          .background(
            color = purpleDark[6],
            shape =
              RoundedCornerShape(
                topStart = 8.dp,
                topEnd = 8.dp,
                bottomStart = 0.dp,
                bottomEnd = 0.dp,
              ),
          )
          .padding(8.dp)
          .pointerInput(Unit) {
            detectTransformGestures { _, pan, _, _ ->
              position += pan
              fgNode = id
            }
          },
        horizontalArrangement = Arrangement.Center,
        verticalAlignment = Alignment.CenterVertically,
      ) {
        CompositionLocalProvider(
          LocalTextStyle provides
            TextStyle(
              fontSize = 14.sp,
              color = mauveDark[11],
              fontWeight = FontWeight.Bold,
              fontFamily = InterVariableFamily.current,
            )
        ) {
          Box(modifier = Modifier.fillMaxWidth()) {
            CollapseButton(
              modifier = Modifier.align(Alignment.CenterStart),
              collapsed = state.collapsed,
              onClick = { state.collapsed = !state.collapsed },
            )
            Box(modifier = Modifier.align(Alignment.Center)) { title() }
          }
        }
      }
      AnimatedVisibility(!(collapsible && state.collapsed)) {
        Row(Modifier.padding(8.dp)) { content() }
      }
    }
  }
}

@Composable
fun CollapseButton(
  modifier: Modifier = Modifier,
  color: Color = mauveDark[11],
  size: Dp = 14.dp,
  collapsed: Boolean = false,
  onClick: () -> Unit = {},
) {
  val rotation by animateFloatAsState(targetValue = if (collapsed) 0f else 90f)
  val interactionSource = remember { MutableInteractionSource() }
  Canvas(
    modifier =
      modifier.size(size).clickable(interactionSource = interactionSource, indication = null) {
        onClick()
      }
  ) {
    val w = size.toPx()
    val h = size.toPx()
    val path =
      Path().apply {
        moveTo(0f, 0f)
        lineTo(0f, h)
        lineTo(w, h / 2f)
        close()
      }

    rotate(degrees = rotation) { drawPath(path, color = color) }
  }
}
