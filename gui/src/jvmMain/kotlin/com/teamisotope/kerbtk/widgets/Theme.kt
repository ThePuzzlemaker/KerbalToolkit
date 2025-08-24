package com.teamisotope.kerbtk.widgets

import androidx.compose.foundation.*
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.focus.focusProperties
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
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.composeunstyled.*
import com.composeunstyled.theme.ColoredIndication
import com.composeunstyled.theme.rememberColoredIndication
import kerbaltoolkit.gui.generated.resources.InterVariable
import kerbaltoolkit.gui.generated.resources.InterVariable_Italic
import kerbaltoolkit.gui.generated.resources.Res
import org.jetbrains.compose.resources.Font

val InterVariableFamily = compositionLocalOf<FontFamily> { error("No KerbTkTheme in hierarchy") }

@Composable
fun KerbTkRoot(content: @Composable () -> Unit) {
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
      TextStyle(
        fontSize = Theme.textSize,
        color = mauveDark[11],
        fontFamily = interVariable,
        lineHeight = Theme.lineHeight,
      ),
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
  textStyle: TextStyle = LocalTextStyle.current,
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
    textStyle = textStyle,
    modifier =
      modifier
        .focusProperties { canFocus = interactive }
        .background(
          if (interactive) mauveDark[0] else mauveDark[2],
          shape = RoundedCornerShape(Theme.cornerRadius),
        )
        .border(
          width = Theme.borderWidth,
          shape = RoundedCornerShape(Theme.cornerRadius),
          color = borderColor ?: (if (interactive) mauveDark[6] else mauveDark[5]),
        )
        .focusRing(
          textFieldInteract,
          width = if (borderColor == null) Theme.borderWidth else 0.dp,
          color = purpleDark[6],
          shape = RoundedCornerShape(Theme.cornerRadius),
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
    TextInput(modifier = Modifier.padding(Theme.framePadding).height(lineHeightDp))
  }
}

@Composable
fun KtkCheckbox(
  icon: @Composable () -> Unit,
  checked: Boolean,
  modifier: Modifier = Modifier,
  onCheckedChange: (Boolean) -> Unit = {},
  interactionSource: MutableInteractionSource = MutableInteractionSource(),
  content: @Composable () -> Unit,
) {
  val indication =
    rememberColoredIndication(hoveredColor = mauveDarkA[3], pressedColor = mauveDarkA[4])
  Row(
    horizontalArrangement = Arrangement.spacedBy(Theme.standardSpacing),
    verticalAlignment = Alignment.CenterVertically,
  ) {
    Checkbox(
      checked = checked,
      checkIcon = icon,
      onCheckedChange = onCheckedChange,
      shape = RoundedCornerShape(4.dp),
      interactionSource = interactionSource,
      indication = NoIndictation,
      modifier =
        modifier
          .background(color = mauveDark[0], shape = RoundedCornerShape(4.dp))
          .border(width = 1.dp, shape = RoundedCornerShape(4.dp), color = mauveDark[6])
          .indication(interactionSource, indication)
          .padding(Theme.framePadding)
          .size(lineHeightDp),
    )
    content()
  }
}

@Composable
fun KtkSelectButton(
  selected: Boolean,
  onClick: () -> Unit,
  modifier: Modifier = Modifier,
  interactionSource: MutableInteractionSource = MutableInteractionSource(),
  borderColor: Color? = null,
  content: @Composable () -> Unit,
) {
  KtkButton(
    onClick = onClick,
    modifier =
      if (selected)
        modifier.background(purpleDark[4], shape = RoundedCornerShape(Theme.cornerRadius))
      else modifier,
    interactionSource = interactionSource,
    borderColor = borderColor,
    indication =
      remember(selected) {
        ColoredIndication(
          hoveredColor = if (selected) purpleDarkA[3] else mauveDarkA[3],
          pressedColor = if (selected) purpleDarkA[4] else mauveDarkA[4],
          focusedColor = Color.Unspecified,
        )
      },
    content = content,
  )
}

@Composable
fun KtkButton(
  onClick: () -> Unit,
  modifier: Modifier = Modifier,
  interactionSource: MutableInteractionSource = MutableInteractionSource(),
  borderColor: Color? = null,
  indication: Indication? = LocalIndication.current,
  content: @Composable () -> Unit,
) {
  val indication =
    indication
      ?: rememberColoredIndication(hoveredColor = mauveDarkA[3], pressedColor = mauveDarkA[4])
  Button(
    interactionSource = interactionSource,
    onClick = onClick,
    indication = indication,
    modifier =
      Modifier.background(color = mauveDark[0], shape = RoundedCornerShape(Theme.cornerRadius))
        .hoverable(interactionSource)
        .then(modifier)
        .border(
          width = Theme.borderWidth,
          shape = RoundedCornerShape(Theme.cornerRadius),
          color = borderColor ?: mauveDark[6],
        ),
    //        .focusRing(
    //          interactionSource = interactionSource,
    //          width = if (borderColor == null) 1.dp else 0.dp,
    //          color = purpleDark[6],
    //          shape = RoundedCornerShape(4.dp),
    //        ) TODO: find a way to only show the focus ring for tab-navigation focus
    contentPadding = PaddingValues(Theme.framePadding),
    shape = RoundedCornerShape(Theme.cornerRadius),
  ) {
    Box(modifier = Modifier.heightIn(Dp.Unspecified, lineHeightDp)) { content() }
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
    contentDescription = contentDescription,
    tint = tint,
  )
}

val lineHeightDp: Dp
  @Composable
  get() {
    val textStyle = LocalTextStyle.current
    val singleLineHeightDp =
      with(LocalDensity.current) {
        textStyle.lineHeight.takeIf { it.isSp }?.toDp() ?: Theme.lineHeight.toDp()
      }
    return singleLineHeightDp
  }

object NoIndictation : Indication {}

object Theme {
  val framePadding: Dp = 4.dp
  val standardSpacing: Dp = 4.dp
  val cornerRadius: Dp = 4.dp
  val borderWidth: Dp = 1.dp
  val windowCornerRadius: Dp = 8.dp
  val windowTitlebarHeight: Dp = 32.dp
  val windowPadding: Dp = 8.dp
  val textSize: TextUnit = 14.sp
  val lineHeight: TextUnit = 16.sp
}
