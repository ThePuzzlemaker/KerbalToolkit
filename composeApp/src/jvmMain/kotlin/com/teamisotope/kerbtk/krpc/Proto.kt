package com.teamisotope.kerbtk.krpc

import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.Serializable
import kotlinx.serialization.protobuf.ProtoNumber

@Serializable
data class ConnectionRequest(
  val type: ConnectionType,
  val clientName: String = "",
  val clientIdentifier: ByteArray = byteArrayOf(),
) {
  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (javaClass != other?.javaClass) return false

    other as ConnectionRequest

    if (type != other.type) return false
    if (clientName != other.clientName) return false
    if (!clientIdentifier.contentEquals(other.clientIdentifier)) return false

    return true
  }

  override fun hashCode(): Int {
    var result = type.hashCode()
    result = 31 * result + clientName.hashCode()
    result = 31 * result + clientIdentifier.contentHashCode()
    return result
  }
}

@Serializable
enum class ConnectionType {
  Rpc,
  Stream;
}

@Serializable
enum class ConnectionStatus {
  Ok,
  MalformedMessage,
  Timeout,
  WrongType;
}

@Serializable
data class ConnectionResponse(
  val status: ConnectionStatus = ConnectionStatus.Ok,
  val message: String = "",
  val clientIdentifier: ByteArray = byteArrayOf()
) {
  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (javaClass != other?.javaClass) return false

    other as ConnectionResponse

    if (status != other.status) return false
    if (message != other.message) return false
    if (!clientIdentifier.contentEquals(other.clientIdentifier)) return false

    return true
  }

  override fun hashCode(): Int {
    var result = status.hashCode()
    result = 31 * result + message.hashCode()
    result = 31 * result + clientIdentifier.contentHashCode()
    return result
  }
}