package com.teamisotope.kerbtk.widgets

import androidx.compose.foundation.*
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.interaction.collectIsHoveredAsState
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.focus.FocusRequester
import androidx.compose.ui.focus.focusProperties
import androidx.compose.ui.focus.focusRequester
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.SolidColor
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.input.key.*
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.*
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.composeunstyled.*
import com.composeunstyled.theme.ColoredIndication
import com.composeunstyled.theme.rememberColoredIndication
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
  val indication = rememberColoredIndication(pressedColor = mauveDarkA[4])
  CompositionLocalProvider(
    LocalTextStyle provides
      TextStyle(fontSize = 14.sp, color = mauveDark[11], fontFamily = interVariable),
    InterVariableFamily provides interVariable,
    LocalIndication provides indication,
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
        .focusProperties { canFocus = interactive }
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
fun KtkSelectButton(
  selected: Boolean,
  onClick: () -> Unit,
  modifier: Modifier = Modifier,
  interactionSource: MutableInteractionSource? = null,
  borderColor: Color? = null,
  content: @Composable () -> Unit,
) {
  KtkButton(
    onClick = onClick,
    modifier =
      if (selected) modifier.background(purpleDark[4], shape = RoundedCornerShape(4.dp))
      else modifier,
    interactionSource = interactionSource,
    borderColor = borderColor,
    indication =
      remember(selected) { ColoredIndication(
        hoveredColor = if (selected) purpleDarkA[4] else mauveDarkA[4],
        pressedColor = if (selected) purpleDarkA[4] else mauveDarkA[4],
        focusedColor = Color.Unspecified
      ) },
    content = content,
  )
}

@Composable
fun KtkButton(
  onClick: () -> Unit,
  modifier: Modifier = Modifier,
  interactionSource: MutableInteractionSource? = null,
  borderColor: Color? = null,
  indication: Indication? = LocalIndication.current,
  content: @Composable () -> Unit,
) {
  val interactionSource = interactionSource ?: remember { MutableInteractionSource() }
  val hovering by interactionSource.collectIsHoveredAsState()
  Button(
    interactionSource = interactionSource,
    onClick = onClick,
    modifier =
      Modifier.background(
          color = if (hovering) mauveDark[3] else mauveDark[0],
          shape = RoundedCornerShape(4.dp),
        )
        .then(modifier)
        .border(width = 1.dp, shape = RoundedCornerShape(4.dp), color = borderColor ?: mauveDark[6])
//        .focusRing(
//          interactionSource = interactionSource,
//          width = if (borderColor == null) 1.dp else 0.dp,
//          color = purpleDark[6],
//          shape = RoundedCornerShape(4.dp),
//        ) TODO: find a way to only show the focus ring for tab-navigation focus
        .hoverable(interactionSource = interactionSource),
    contentPadding = PaddingValues(6.dp),
    shape = RoundedCornerShape(4.dp),
    indication = indication,
  ) {
    Box(modifier = Modifier.height(lineHeightDp)) { content() }
  }
}

@Composable
fun KtkIcon(
  imageVector: ImageVector,
  modifier: Modifier = Modifier,
  contentDescription: String? = null,
  tint: Color = mauveDark[11],
) {
  Icon(
    imageVector = imageVector,
    modifier = modifier.focusProperties { canFocus = false },
    contentDescription = null,
    tint = tint,
  )
}

val lineHeightDp: Dp
  @Composable
  get() {
    val textStyle = LocalTextStyle.current
    val singleLineHeightDp =
      with(LocalDensity.current) { textStyle.lineHeight.takeIf { it.isSp }?.toDp() ?: 16.sp.toDp() }
    return singleLineHeightDp
  }
