package com.teamisotope.kerbtk.krpc

import io.ktor.network.selector.*
import io.ktor.network.sockets.*
import io.ktor.utils.io.*
import kotlinx.coroutines.Dispatchers
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.KSerializer
import kotlinx.serialization.builtins.serializer
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
      } catch (e: Exception) {
        e.printStackTrace()
      }
    }
  }
}
