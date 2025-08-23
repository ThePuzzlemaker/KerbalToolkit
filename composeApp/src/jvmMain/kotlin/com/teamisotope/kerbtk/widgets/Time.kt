package com.teamisotope.kerbtk.widgets

import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.widthIn
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import com.adamglin.PhosphorIcons
import com.adamglin.phosphoricons.Regular
import com.adamglin.phosphoricons.regular.Timer
import com.composables.core.HorizontalSeparator
import com.composeunstyled.Icon
import com.composeunstyled.LocalTextStyle
import kotlin.math.absoluteValue
import kotlin.time.Duration
import kotlin.time.Duration.Companion.days
import kotlin.time.Duration.Companion.hours
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.minutes
import kotlin.time.Duration.Companion.seconds

enum class TimeInput {
  UT,
  GET,
}

enum class TimeDisplay() {
  Dhms {
    override fun next() = Hms
  },
  Hms {
    override fun next() = Sec
  },
  Sec {
    override fun next() = Dhms
  };

  abstract fun next(): TimeDisplay
}

fun parseDhmsDuration(str: String, allowNegative: Boolean = false): Duration? {
  val res = Duration.parseOrNull(str.trim()) ?: return null
  if (!allowNegative && res.isNegative()) return null
  return res
}

fun parseDhmsTime(str: String, allowNegative: Boolean = false): Duration? {
  var s = str.trim()

  if (s.isEmpty()) return null

  val neg = s.startsWith("-(") && s.endsWith(")")
  if (allowNegative && neg) s = s.substring(2..<(s.length - 1))
  else if (!allowNegative && neg) return null

  val (timeStr, millisStr) = s.split('.', limit = 2).let { it[0] to it.getOrNull(1) }

  val parts =
    try {
      timeStr.split(":").map { it.toLong() }
    } catch (_: NumberFormatException) {
      return null
    }
  if (parts.size !in 3..4) return null

  val d = if (parts.size == 4) parts[0] else 0
  val h = if (parts.size == 4) parts[1] else parts[0]
  val m = if (parts.size == 4) parts[2] else parts[1]
  val sec = if (parts.size == 4) parts[3] else parts[2]
  val millis =
    when (millisStr?.length ?: 0) {
      0 -> 0
      1 -> 100 * (millisStr?.toLongOrNull() ?: return null)
      2 -> 10 * (millisStr?.toLongOrNull() ?: return null)
      3 -> 1 * (millisStr?.toLongOrNull() ?: return null)
      else -> return null
    }

  var dur = d.days + h.hours + m.minutes + sec.seconds + millis.milliseconds
  if (neg) dur = -dur
  return dur
}

fun parseSecTime(str: String, allowNegative: Boolean = false): Duration? {
  var s = str.trim()

  if (str.isEmpty()) return null

  val neg = s.startsWith("-")
  if (allowNegative && neg) s = s.drop(1) else if (!allowNegative && neg) return null

  val input = s.split('.')
  if (input.size !in 1..2) return null

  val sec = input[0].toLongOrNull() ?: return null
  val millisStr = input.getOrNull(1)
  val millis =
    when (millisStr?.length ?: 0) {
      0 -> 0
      1 -> 100 * (millisStr?.toLongOrNull() ?: return null)
      2 -> 10 * (millisStr?.toLongOrNull() ?: return null)
      3 -> 1 * (millisStr?.toLongOrNull() ?: return null)
      else -> return null
    }

  var dur = sec.seconds + millis.milliseconds
  if (neg) dur = -dur
  return dur
}

fun Duration.toStringDhms(): String {
  return "%s%03d:%02d:%02d:%02d.%03d%s"
    .format(
      if (this.isNegative()) "-(" else "",
      this.inWholeDays.absoluteValue,
      this.inWholeHours.absoluteValue % 24L,
      this.inWholeMinutes.absoluteValue % 60L,
      this.inWholeSeconds.absoluteValue % 60L,
      this.inWholeMilliseconds.absoluteValue % 1000L,
      if (this.isNegative()) ")" else "",
    )
}

fun Duration.toStringHms(): String {
  return "%s%02d:%02d:%02d.%03d%s"
    .format(
      if (this.isNegative()) "-(" else "",
      this.inWholeHours.absoluteValue,
      this.inWholeMinutes.absoluteValue % 60L,
      this.inWholeSeconds.absoluteValue % 60L,
      this.inWholeMilliseconds.absoluteValue % 1000L,
      if (this.isNegative()) ")" else "",
    )
}

fun Duration.toStringSec(): String {
  val builder = StringBuilder()
  if (this.isNegative()) builder.append("-")
  builder.append(this.inWholeSeconds.absoluteValue)
  val millis = this.inWholeMilliseconds.absoluteValue % 1000L
  if (millis != 0L) builder.append(".", millis) else builder.append(".000")
  return builder.toString()
}

fun Duration.toStringTime(kind: TimeDisplay): String {
  return when (kind) {
    TimeDisplay.Dhms -> this.toStringDhms()
    TimeDisplay.Hms -> this.toStringHms()
    TimeDisplay.Sec -> this.toStringSec()
  }
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun TimeInput(
  value: Duration,
  modifier: Modifier = Modifier,
  interactive: Boolean = true,
  allowNegative: Boolean = false,
  onChanged: (Duration) -> Unit = {},
) {
  var focused by remember { mutableStateOf(false) }
  var display by remember { mutableStateOf(TimeDisplay.Dhms) }
  var buffer by remember { mutableStateOf(value.toStringTime(display)) }

  LaunchedEffect(value) { if (!focused) buffer = value.toStringTime(display) }

  val parsed by remember {
    derivedStateOf {
      parseDhmsDuration(buffer, allowNegative)
        ?: parseDhmsTime(buffer, allowNegative)
        ?: parseSecTime(buffer, allowNegative)
    }
  }

  LaunchedEffect(parsed, interactive, focused) {
    if (interactive && parsed != null && parsed != value) {
      onChanged(parsed!!)
    }
  }

  LaunchedEffect(focused, parsed, display) {
    if (!focused && parsed != null) {
      buffer = parsed!!.toStringTime(display)
    }
  }

  val focusManager = LocalFocusManager.current
  Row(modifier = modifier, horizontalArrangement = Arrangement.spacedBy(4.dp)) {
    KtkTextField(
      value = buffer,
      modifier =
        Modifier.widthIn(Dp.Unspecified, 196.dp).onFocusChanged { focusState ->
          focused = focusState.hasFocus
        },
      singleLine = true,
      interactive = interactive,
      onValueChange = { buffer = it },
      textStyle = LocalTextStyle.current.copy(fontFamily = FontFamily.Monospace),
      borderColor =
        if (parsed == null) {
          redDark[6]
        } else {
          null
        },
    )

    KtkButton(onClick = { display = display.next() }) {
      Icon(
        imageVector = PhosphorIcons.Regular.Timer,
        contentDescription = null,
        tint = mauveDark[11],
      )
    }
  }
}

@Composable
fun KtkHorizontalSeparator(modifier: Modifier = Modifier) {
  HorizontalSeparator(mauveDark[6], modifier = Modifier.padding(top = Theme.standardSpacing, bottom = Theme.standardSpacing))
}
