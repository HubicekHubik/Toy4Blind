package com.example.toysender

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import kotlin.collections.set
import kotlin.collections.toMutableList
import kotlin.collections.toMutableMap

class MyViewModel(application: Application) : AndroidViewModel(application) {
    val bleManager1 = MyBleManager(application)
    val bleManager2 = MyBleManager(application)

    val activeToyIndex = MutableLiveData<Int>(0)

    val connectionState1 = MutableLiveData(BleState.DISCONNECTED)
    val connectionState2 = MutableLiveData(BleState.DISCONNECTED)

    private val _isScanEnabled = MutableLiveData<Boolean>(true)
    val isScanEnabled: LiveData<Boolean> get() = _isScanEnabled

    fun setScanEnabled(enabled: Boolean) {
        _isScanEnabled.postValue(enabled)
    }
    private val toysData = mutableMapOf<Int, Map<String, Map<String, List<SdFile>>>>(
        0 to emptyMap(),
        1 to emptyMap()
    )
    private val _groupedFiles = MutableLiveData<Map<String, Map<String, List<SdFile>>>>(emptyMap())
    val groupedFiles: LiveData<Map<String, Map<String, List<SdFile>>>> = _groupedFiles
    val expandedCategory = MutableLiveData<String?>(null)

    val expandedFolder = MutableLiveData<String?>(null)

    fun getActiveManager(): MyBleManager = if (activeToyIndex.value == 0) bleManager1 else bleManager2

    fun getActiveConnectionState(): LiveData<BleState> {
        return if (activeToyIndex.value == 0) connectionState1 else connectionState2
    }
    fun swapToys() {
        val newIndex = if (activeToyIndex.value == 0) 1 else 0
        activeToyIndex.value = newIndex

        _groupedFiles.value = toysData[newIndex] ?: emptyMap()

        getActiveManager().sendData(byteArrayOf(MyBleManager.MT_SWITCH_TOY))
        getActiveManager().sendData(byteArrayOf(MyBleManager.MT_REQUEST_SD_DATA))
    }
    fun clearToyData(toyIndex: Int) {
        toysData[toyIndex] = emptyMap()
        if (activeToyIndex.value == toyIndex) {
            _groupedFiles.postValue(emptyMap())
        }
    }
    fun toggleCategory (categName: String) {
        if (expandedCategory.value == categName) {
            expandedCategory.value = null
        } else {
            expandedCategory.value = categName
        }
    }
    fun toggleFolder(folderName: String) {
        if (expandedFolder.value == folderName) {
            expandedFolder.value = null
        } else {
            expandedFolder.value = folderName
        }
    }
    fun parseAndAddPath(toyIndex: Int, path: String) {
        val parts = path.split("/")
        if (parts.size < 2) return

        val category = parts[0]
        val folder = parts[1]
        val fileName = parts.getOrNull(2)

        val currentToyMap = toysData[toyIndex]?.toMutableMap() ?: mutableMapOf()

        val foldersInCategory = currentToyMap[category]?.toMutableMap() ?: mutableMapOf()

        val fileList = foldersInCategory[folder]?.toMutableList() ?: mutableListOf()

        if (!fileName.isNullOrBlank()) {
            val newFile = SdFile(category, folder, fileName, path)

            if (!fileList.any { it.fullPath == path }) {
                fileList.add(newFile)
            }
        }

        foldersInCategory[folder] = fileList
        currentToyMap[category] = foldersInCategory

        toysData[toyIndex] = currentToyMap

        if (activeToyIndex.value == toyIndex) {
            _groupedFiles.postValue(currentToyMap)
        }
    }
}