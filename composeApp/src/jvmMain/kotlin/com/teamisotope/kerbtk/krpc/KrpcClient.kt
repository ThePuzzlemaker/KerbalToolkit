package com.teamisotope.kerbtk.krpc

import io.ktor.network.selector.*
import io.ktor.network.sockets.*
import io.ktor.utils.io.*
import java.io.EOFException
import kotlinx.coroutines.Dispatchers
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.KSerializer
import kotlinx.serialization.builtins.serializer
import kotlinx.serialization.decodeFromByteArray
import kotlinx.serialization.protobuf.ProtoBuf

class KrpcClient
private constructor(
  private val rpc: Socket,
  private val readChannel: ByteReadChannel,
  private val writeChannel: ByteWriteChannel,
) {
  companion object {
    suspend fun connect(clientName: String, host: String, port: Int): KrpcClient {
      val rpc =
        aSocket(ActorSelectorManager(Dispatchers.IO)).tcp().connect(InetSocketAddress(host, port))
      val readChannel = rpc.openReadChannel()
      val writeChannel = rpc.openWriteChannel(autoFlush = true)

      val client = KrpcClient(rpc, readChannel, writeChannel)

      client.send(
        ConnectionRequest.serializer(),
        ConnectionRequest(ConnectionType.Rpc, clientName, byteArrayOf()),
      )
      val res = client.recv(ConnectionResponse.serializer())
      if (res.status != ConnectionStatus.Ok) {
        throw KrpcConnectException(res.message)
      }
      return client
    }
  }

  @OptIn(ExperimentalSerializationApi::class)
  private suspend fun <T> send(serializer: KSerializer<T>, message: T) {
    val arr = ProtoBuf.encodeToByteArray(serializer, message)
    writeChannel.writeByteArray(ProtoBuf.encodeToByteArray(Int.serializer(), arr.size))
    writeChannel.writeByteArray(arr)
  }

  @OptIn(ExperimentalSerializationApi::class)
  private suspend fun <T> recv(serializer: KSerializer<T>): T {
    val lenData = arrayListOf<Byte>()
    var size = 0
    while (true) {
      lenData.add(readChannel.readByte())
      try {
        size = ProtoBuf.decodeFromByteArray(Int.serializer(), lenData.toByteArray())
        break
      } catch (_: EOFException) {
        continue
      }
    }

    val buf = readChannel.readByteArray(size)
    val res = ProtoBuf.decodeFromByteArray(serializer, buf)
    return res
  }
}

data class KrpcConnectException(val serverMessage: String) :
  Exception("Failed to connect to kRPC: $serverMessage") {}
