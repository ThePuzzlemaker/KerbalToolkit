package com.teamisotope.kerbtk.widgets

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.SolidColor
import androidx.compose.ui.input.key.*
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.*
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.composeunstyled.*
import kerbaltoolkit.composeapp.generated.resources.InterVariable
import kerbaltoolkit.composeapp.generated.resources.InterVariable_Italic
import kerbaltoolkit.composeapp.generated.resources.Res
import org.jetbrains.compose.resources.Font

val InterVariableFamily = compositionLocalOf<FontFamily> { error("No KerbTkTheme in hierarchy") }

@Composable
fun KerbTkTheme(content: @Composable () -> Unit) {
  val fonts = mutableListOf<Font>()
  for (wght in 100..900 step 100) {
    fonts.add(
      Font(
        Res.font.InterVariable,
        style = FontStyle.Normal,
        weight = FontWeight(wght),
        variationSettings = FontVariation.Settings(FontVariation.weight(wght)),
      )
    )
    fonts.add(
      Font(
        Res.font.InterVariable_Italic,
        style = FontStyle.Italic,
        weight = FontWeight(wght),
        variationSettings = FontVariation.Settings(FontVariation.weight(wght)),
      )
    )
  }
  val interVariable = FontFamily(fonts)
  CompositionLocalProvider(
    LocalTextStyle provides
      TextStyle(fontSize = 14.sp, color = mauveDark[11], fontFamily = interVariable),
    InterVariableFamily provides interVariable,
  ) {
    content()
  }
}

@Composable
fun KtkTextField(
  value: String,
  modifier: Modifier = Modifier,
  singleLine: Boolean = false,
  interactive: Boolean = true,
  keyboardActions: KeyboardActions = KeyboardActions.Default,
  interactionSource: MutableInteractionSource? = null,
  borderColor: Color? = null,
  onValueChange: (String) -> Unit,
) {
  val textFieldInteract = interactionSource ?: remember { MutableInteractionSource() }
  var focused by remember { mutableStateOf(false) }
  val focusManager = LocalFocusManager.current

  TextField(
    interactionSource = textFieldInteract,
    value = value,
    singleLine = singleLine,
    cursorBrush = SolidColor(mauveDark[11]),
    onValueChange = onValueChange,
    textStyle = TextStyle(fontFamily = FontFamily.Monospace, color = mauveDark[11]),
    modifier =
      modifier
        .background(
          if (interactive) mauveDark[0] else mauveDark[2],
          shape = RoundedCornerShape(4.dp),
        )
        .border(
          width = 1.dp,
          shape = RoundedCornerShape(4.dp),
          color = borderColor ?: (if (interactive) mauveDark[6] else mauveDark[5]),
        )
        .focusRing(
          textFieldInteract,
          width = if (borderColor == null) 1.dp else 0.dp,
          color = purpleDark[6],
          shape = RoundedCornerShape(4.dp),
        )
        .onKeyEvent { keyEvent ->
          if (
            focused &&
              keyEvent.type == KeyEventType.KeyUp &&
              (keyEvent.key == Key.Escape || keyEvent.key == Key.Enter)
          ) {
            focusManager.clearFocus()
            return@onKeyEvent true
          }
          return@onKeyEvent false
        }
        .onFocusChanged { focusState -> focused = focusState.hasFocus },
    editable = interactive,
    keyboardActions = keyboardActions,
  ) {
    TextInput(modifier = Modifier.padding(6.dp))
  }
}

@Composable
fun KtkButton(onClick: () -> Unit, modifier: Modifier = Modifier, content: @Composable () -> Unit) {
  Button(
    onClick = onClick,
    modifier =
      modifier
        .background(color = mauveDark[0], shape = RoundedCornerShape(4.dp))
        .border(width = 1.dp, shape = RoundedCornerShape(4.dp), color = mauveDark[6])
        .padding(6.dp),
  ) {
    Box(modifier = Modifier.height(lineHeightDp)) { content() }
  }
}

val lineHeightDp: Dp
  @Composable
  get() {
    val textStyle = LocalTextStyle.current
    val singleLineHeightDp =
      with(LocalDensity.current) { textStyle.lineHeight.takeIf { it.isSp }?.toDp() ?: 16.sp.toDp() }
    return singleLineHeightDp
  }
