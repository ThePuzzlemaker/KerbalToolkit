package com.teamisotope.kerbtk.widgets

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.core.animateDpAsState
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.expandVertically
import androidx.compose.animation.shrinkVertically
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
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.zIndex
import com.adamglin.PhosphorIcons
import com.adamglin.phosphoricons.Bold
import com.adamglin.phosphoricons.bold.X
import com.composeunstyled.LocalTextStyle

private val ForegroundNode =
  compositionLocalOf<MutableState<String?>> { error("No window canvas present") }

@Composable
fun FloatingWindowCanvas(content: @Composable () -> Unit) {
  val fgNode = remember { mutableStateOf<String?>(null) }
  Box(Modifier.fillMaxSize().background(mauveDark[0])) {
    CompositionLocalProvider(ForegroundNode provides fgNode) { content() }
  }
}

class WindowState {
  var collapsed: Boolean by mutableStateOf(false)
  var open: Boolean by mutableStateOf(true)
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun FloatingWindow(
  title: @Composable () -> Unit,
  /// A unique identifier (within the WindowCanvas) of this window.
  id: String,
  modifier: Modifier = Modifier,
  state: WindowState = WindowState(),
  collapsible: Boolean = true,
  closable: Boolean = true,
  onClose: () -> Unit = {},
  content: @Composable () -> Unit,
) {
  var position by remember { mutableStateOf(Offset.Zero) }
  var fgNode by ForegroundNode.current
  // Box here allows us to constrain the size of the window without breaking collapsing.

  Box(
    modifier
      .graphicsLayer(translationX = position.x, translationY = position.y)
      .zIndex(
        if (fgNode == id) {
          100f
        } else {
          0f
        }
      )
  ) {
    AnimatedVisibility(
      visible = state.open,
      enter = expandVertically(expandFrom = Alignment.Top),
      exit = shrinkVertically(shrinkTowards = Alignment.Top),
    ) {
      Column(
        Modifier.background(color = mauveDark[2], shape = RoundedCornerShape(size = 8.dp))
          .border(width = 1.dp, color = mauveDark[5], shape = RoundedCornerShape(size = 8.dp))
          .onClick { fgNode = id }
      ) {
        val bottomRadius by animateDpAsState(if (state.collapsed) 8.dp else 0.dp)
        Row(
          Modifier.height(height = 32.dp)
            .fillMaxWidth()
            .background(
              color = purpleDark[6],
              shape =
                RoundedCornerShape(
                  topStart = 8.dp,
                  topEnd = 8.dp,
                  bottomStart = bottomRadius,
                  bottomEnd = bottomRadius,
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
              if (closable) {
                Image(
                  imageVector = PhosphorIcons.Bold.X,
                  contentDescription = null,
                  contentScale = ContentScale.Fit,
                  colorFilter = ColorFilter.tint(mauveDark[11]),
                  modifier =
                    Modifier.height(18.dp).align(Alignment.CenterEnd).clickable {
                      state.open = false
                      onClose()
                    },
                )
              }
            }
          }
        }
        AnimatedVisibility(!(collapsible && state.collapsed)) {
          Row(Modifier.padding(8.dp)) { content() }
        }
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
