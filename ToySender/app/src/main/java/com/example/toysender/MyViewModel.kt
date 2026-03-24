package com.example.toysender

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import kotlin.collections.set
import kotlin.collections.toMutableList
import kotlin.collections.toMutableMap

class MyViewModel : ViewModel() {
    val connectionState = MutableLiveData(BleState.DISCONNECTED)
    private val _groupedFiles = MutableLiveData<Map<String, Map<String, List<SdFile>>>>(emptyMap())
    val groupedFiles: LiveData<Map<String, Map<String, List<SdFile>>>> = _groupedFiles
    val expandedCategory = MutableLiveData<String?>(null)

    val expandedFolder = MutableLiveData<String?>(null)


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
    fun parseAndAddPath(path: String) {
        val parts = path.split("/")
        if (parts.size < 2) return

        val category = parts[0]
        val folder = parts[1]
        val fileName = parts.getOrNull(2)

        val currentMap = _groupedFiles.value?.toMutableMap() ?: mutableMapOf()

        val foldersInCategory = currentMap[category]?.toMutableMap() ?: mutableMapOf()

        val fileList = foldersInCategory[folder]?.toMutableList() ?: mutableListOf()

        if (!fileName.isNullOrBlank()) {
            val newFile = SdFile(category, folder, fileName, path)
            // Prevence duplicit
            if (!fileList.any { it.fullPath == path }) {
                fileList.add(newFile)
            }
        }

        foldersInCategory[folder] = fileList
        currentMap[category] = foldersInCategory

        _groupedFiles.value = currentMap
    }

    fun clearAllData() {
        _groupedFiles.value = emptyMap()
    }
}