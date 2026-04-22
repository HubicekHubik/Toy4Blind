package com.example.toysender

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic
import android.content.Context
import android.net.Uri
import android.provider.OpenableColumns
import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import no.nordicsemi.android.ble.BleManager
import no.nordicsemi.android.ble.callback.DataReceivedCallback
import no.nordicsemi.android.ble.data.Data
import java.util.Locale
import kotlin.collections.chunked
import kotlin.collections.toByteArray
import kotlin.math.sqrt

enum class BleState { DISCONNECTED, SCANNING, CONNECTED, ERROR }

data class BatteryInfo(
    val voltageMv: Int,
    val isCharging: Boolean,
    val isCharged: Boolean,
    val isDeviceOn: Boolean
)
class MyBleManager(context: Context) : BleManager(context), DataReceivedCallback {
    private val _batteryInfo = MutableLiveData<BatteryInfo>()
    val batteryInfo: LiveData<BatteryInfo> get() = _batteryInfo
    private val _isToyOn = MutableLiveData<Boolean>(false)
    val isToyOn: LiveData<Boolean> get() = _isToyOn
    private val UART_SERVICE_UUID = java.util.UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
    private val UART_RX_CHAR_UUID = java.util.UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e")
    private val UART_TX_CHAR_UUID = java.util.UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e")
    private var rxCharacteristic: BluetoothGattCharacteristic? = null
    private var txCharacteristic: BluetoothGattCharacteristic? = null

    val debugData = MutableLiveData<String>()

    var onPathReceived: ((String) -> Unit)? = null

    override fun getGattCallback(): BleManagerGattCallback = object : BleManagerGattCallback() {
        override fun isRequiredServiceSupported(gatt: BluetoothGatt): Boolean {
            val service = gatt.getService(UART_SERVICE_UUID)
            rxCharacteristic = service?.getCharacteristic(UART_RX_CHAR_UUID)
            txCharacteristic = service?.getCharacteristic(UART_TX_CHAR_UUID)

            val properties = rxCharacteristic?.properties ?: 0
            val canWrite = (properties and BluetoothGattCharacteristic.PROPERTY_WRITE != 0) ||
                    (properties and BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE != 0)
            val canNotify = (txCharacteristic?.properties ?: 0) and BluetoothGattCharacteristic.PROPERTY_NOTIFY != 0
            return rxCharacteristic != null && txCharacteristic != null && canWrite && canNotify
        }

        override fun onServicesInvalidated() { }
        override fun onDeviceDisconnected() {

        }
    }

    fun sendData(data: ByteArray) {
        if (rxCharacteristic == null) {
            Log.e("BLE", "Nelze odeslat data - charakteristika nenalezena!")
            return
        }

        writeCharacteristic(
            rxCharacteristic,
            data,
            BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        )
            .fail { device, status ->
                Log.e("BLE", "Odesílání selhalo! Status: $status")
            }
            .enqueue()
    }

    override fun onDataReceived(device: BluetoothDevice, data: Data) {
        Log.d("BLE_RAW", "Přišla data: ${data.size()} bajtů, první bajt: ${data.getByte(0)}")
        val type = data.getByte(0) ?: return

        when (type) {
            MT_RECV_SD_DATA -> {
                val path = data.getStringValue(1) ?: return
                Log.d("BLE_PATH", "Path: $path")
                onPathReceived?.invoke(path)
            }
            MT_CLEAR_DATA -> {
                onPathReceived?.invoke("CLEAR_ALL_FILES_SIGNAL")

            }
            MT_DEBUG_DATA -> {
                val rawData = data.value
                if (rawData != null && rawData.size > 1) {
                    val tableText = String(rawData, 1, rawData.size - 1, Charsets.UTF_8)

                    debugData.postValue(tableText)
                }
            }
            MT_RECV_POW_DATA -> {
                val raw = data.value ?: return
                if (raw.size >= 6) {
                    val vBat = ((raw[2].toInt() and 0xFF) shl 8) or (raw[1].toInt() and 0xFF)
                    val charging = raw[3].toInt() != 0
                    val charged = raw[4].toInt() != 0
                    val isOn = raw[5].toInt() != 0
                    Log.d("BatteryInfo", "BatteryInfo: mV:${vBat} chraging:${charging} charged:${charged} isOn:${isOn}")
                    _batteryInfo.postValue(BatteryInfo(vBat, charging, charged, isOn))
                }
            }
        }
    }

    override fun initialize() {
        requestMtu(247)
            .with { device, mtu -> Log.d("BLE_MTU", "New MTU set to: $mtu pro ${device.name}") }
            .enqueue()

        setNotificationCallback(txCharacteristic).with(this)
        enableNotifications(txCharacteristic).enqueue()

        sendData(byteArrayOf(MT_REQUEST_SD_DATA))
    }

    @SuppressLint("Range")
    fun getFileName(uri: Uri): String {
        var result: String? = null
        if (uri.scheme == "content") {
            val cursor = context.contentResolver.query(uri, null, null, null, null)
            try {
                if (cursor != null && cursor.moveToFirst()) {
                    result = cursor.getString(cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME))
                }
            } finally {
                cursor?.close()
            }
        }
        if (result == null) {
            result = uri.path
            val cut = result?.lastIndexOf('/')
            if (cut != null && cut != -1) {
                result = result?.substring(cut + 1)
            }
        }
        return result ?: "Neznámý soubor"
    }

    private fun createheader(uri: Uri?, currentDir: String): ByteArray? {
        val uri = uri ?: return null

        return try {
            context.contentResolver.openInputStream(uri)?.use { inputStream ->
                val fileBytes = inputStream.readBytes()
                val fileLen = fileBytes.size
                val fileName = getFileName(uri)//.removeSuffix(".wav")
                val filePath = "$currentDir/$fileName"
                var filePathLen = filePath.length

                Log.d("TRANSFER", "MSG_TYPE: $MT_FILE, File_size: $fileLen, Velikost_jména: $filePathLen, Soubor: $filePath,")

                val header = ByteArray(244)
                header[0] = MT_FILE

                header[1] = (fileLen and 0xFF).toByte()
                header[2] = ((fileLen shr 8) and 0xFF).toByte()
                header[3] = ((fileLen shr 16) and 0xFF).toByte()
                header[4] = ((fileLen shr 24) and 0xFF).toByte()
                if (filePathLen > 128) filePathLen = 127
                header[5] = (filePathLen and 0xFF).toByte()
                header[6] = ((filePathLen shr 8) and 0xFF).toByte()

                // File name (first 4 bytes)
                val nameBytes = filePath.toByteArray(Charsets.UTF_8)
                val actualBytesToCopy = if (nameBytes.size > filePathLen) filePathLen else nameBytes.size
                for (i in 0 until actualBytesToCopy) {
                    header[7 + i] = if (i < nameBytes.size) nameBytes[i] else 0
                }

                header
            }
        } catch (e: Exception) {
            Log.e("TRANSFER", "Chyba při čtení souboru", e)
            null
        }
    }

    private fun devide_and_send(uri: Uri?) {
        val uri = uri ?: return
        try {
            context.contentResolver.openInputStream(uri)?.use { inputStream ->
                val fileBytes = inputStream.readBytes()
                val chunkSize = 243
                val payloads = fileBytes.toList().chunked(chunkSize).map { chunk ->
                    byteArrayOf(MT_FILE_TRANSFER) + chunk.toByteArray()
                }
                for (payload in payloads) {
                    sendData(payload)
                }
            }
            sendData(byteArrayOf(MT_DEC_SPEED))
        } catch (e: Exception) {
            Log.e("DATA_TRANSFER", "Chyba při posíláni audio dat", e)
        }
    }

    fun startSendingFile(uri: Uri?, currentDir: String) {
        if (uri == null) return
        val header = createheader(uri, currentDir)
        if (header != null) {
            sendData(header)
            sendData(byteArrayOf(MT_INC_SPEED))
        }
        devide_and_send(uri)
    }

    fun requestDelete(path: String, cmd: Byte) {
        val pathBytes = path.toByteArray(Charsets.UTF_8)
        if (cmd != MT_DELETE_SD_CATEGORY) {
            pathBytes[1] = 0x5F.toByte()
        }
        Log.d("BLE_PATH", "Path: ${String(pathBytes, Charsets.UTF_8)}")
        sendData(byteArrayOf(cmd) + pathBytes)
    }
    fun requestAdd(path: String, cmd: Byte) {
        val pathBytes = path.toByteArray(Charsets.UTF_8)
        pathBytes[1] = 0x5F.toByte()
        sendData(byteArrayOf(cmd) + pathBytes)
    }
    fun requestRename(oldPath: String, newName: String, cmd: Byte) {
        val oldPathBytes = oldPath.toByteArray(Charsets.UTF_8)
        if (oldPathBytes.size > 1 && oldPathBytes[1] == 0x2F.toByte()) {
            oldPathBytes[1] = 0x5F.toByte()
        }
        val newNameBytes = newName.toByteArray(Charsets.UTF_8)

        val totalData = ByteArray(1 + 1 + oldPathBytes.size + newNameBytes.size)

        totalData[0] = cmd
        totalData[1] = oldPathBytes.size.toByte()

        oldPathBytes.copyInto(totalData, destinationOffset = 2)

        newNameBytes.copyInto(totalData, destinationOffset = 2 + oldPathBytes.size)

        Log.d("BLE_RENAME", "Odesílám: ${totalData.toString()}")
        sendData(totalData)
    }

    fun setToyState(isOn: Boolean) {
        _isToyOn.postValue(isOn)
    }

    fun toggleToy() {
        val currentState = _isToyOn.value ?: false
        val newState = !currentState

        if(currentState){
            sendData(byteArrayOf(MT_LSM6DSL_ON))
        } else {
            sendData(byteArrayOf(MT_LSM6DSL_OFF))
        }
        sendData(byteArrayOf(MT_REQ_LASTBAT))
        _isToyOn.value = newState
    }
    companion object {
        const val MT_REQUEST_SD_DATA: Byte = 0x6D.toByte()
        const val MT_RECV_SD_DATA: Byte = 0x7D.toByte()
        const val MT_RECV_POW_DATA: Byte = 0x9D.toByte()
        const val MT_FILE: Byte = 0xFE.toByte()

        const val MT_INC_SPEED: Byte = 0x16.toByte()
        const val MT_DEC_SPEED: Byte = 0xD6.toByte()
        const val MT_G_MODE_CHANGE: Byte = 0xC6.toByte()
        const val MT_FILE_TRANSFER: Byte = 0xF6.toByte()
        const val MT_DELETE_SD_FILE: Byte = 0xDF.toByte()
        const val MT_DELETE_SD_CATEGORY: Byte = 0xDC.toByte()
        const val MT_RENAME_SD_CATEGORY: Byte = 0xEC.toByte()
        const val MT_RENAME_SD_FILE: Byte = 0xEF.toByte()
        const val MT_RENAME_SD_FOLDER: Byte = 0xED.toByte()
        const val MT_SWITCH_TOY: Byte = 0x66.toByte()
        const val MT_REQ_LASTBAT: Byte = 0xEB.toByte()

        const val MT_ADD_SD_FILE: Byte = 0xAF.toByte()
        const val MT_ADD_DIR: Byte =  0xAD.toByte()
        const val MT_CLEAR_DATA: Byte = 0xCD.toByte()

        const val MSG_TYPE_TOY_SWITCH:  Byte = 0x67.toByte()
        const val MT_LSM6DSL_ON:        Byte = 0x6A.toByte()
        const val MT_LSM6DSL_OFF:       Byte = 0x6F.toByte()
        const val MT_BT_DISCONNECT:     Byte = 0xBE.toByte()
        const val MSG_TYPE_TURNOFF:     Byte = 0x0F.toByte()
        const val MT_VOL_UP:            Byte = 0x08.toByte()
        const val MT_VOL_DOWN:          Byte = 0x09.toByte()

        const val MT_VOL_MUTE:          Byte = 0x0A.toByte()

        const val MSG_TYPE_SOUNDSET:    Byte = 0x6E.toByte()
        const val MT_CHANGE_CATEGORY:   Byte = 0xCC.toByte()

        const val MT_DEBUG_DATA:		Byte = 0xDB.toByte()
    }
}