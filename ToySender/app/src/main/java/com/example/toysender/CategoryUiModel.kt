package com.example.toysender

data class CategoryUiModel(
    val name: String,
    val folders: Map<String, List<SdFile>>,
    val isExpanded: Boolean,
    val expandedFolderName: String?
)