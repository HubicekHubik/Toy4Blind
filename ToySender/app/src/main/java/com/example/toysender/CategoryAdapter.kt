package com.example.toysender

import android.content.Context
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.accessibility.AccessibilityEvent
import android.view.accessibility.AccessibilityManager
import android.view.accessibility.AccessibilityNodeInfo
import android.widget.LinearLayout
import android.widget.ImageView
import android.widget.TextView
import androidx.core.content.ContentProviderCompat.requireContext
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import java.util.logging.Logger

class CategoryAdapter(
    val onCategoryClick: (String) -> Unit,
    val onFolderClick: (String) -> Unit,
    val onFileRename: (SdFile) -> Unit,
    val onFileDelete: (SdFile) -> Unit,
    val onAddFileClick: () -> Unit,
    val onFolderLongClickRename: (String, String, String) -> Unit,
    val onFolderLongClickDelete: (String) -> Unit,
    val onCategLongClickDelete: (String) -> Unit,
    val onCategLongClickRename: (String) -> Unit,
    val onAddFolderClick: (String) -> Unit,
    val onAddCategoryClick: () -> Unit,
) : ListAdapter<CategoryUiModel, CategoryAdapter.ViewHolder>(DiffCallback()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.item_category, parent, false)
        return ViewHolder(view)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        val item = getItem(position)
        holder.bind(item)
    }

    inner class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
        private val title: TextView = view.findViewById(R.id.categoryTitle)
        private val container: LinearLayout = view.findViewById(R.id.categoryContainer)
        private val folderArrow: ImageView = view.findViewById(R.id.folderArrow)

        fun bind(category: CategoryUiModel) {
            // Reset state for recycled view
            title.visibility = View.VISIBLE
            folderArrow.visibility = View.VISIBLE
            container.visibility = View.VISIBLE
            container.removeAllViews()
            itemView.setOnClickListener(null)

            if (category.name == "[ADD_NEW_CATEGORY]") {
                title.visibility = View.GONE
                folderArrow.visibility = View.GONE
                container.visibility = View.VISIBLE

                val addView = LayoutInflater.from(container.context)
                    .inflate(R.layout.add_category_button, container, false)

                addView.setOnClickListener {
                    val categoryCount = currentList.count { it.name != "[ADD_NEW_CATEGORY]" }
                    Log.d("CategoryAdapter", "categoryCount: $categoryCount")
                    if (categoryCount < 8) {
                        onAddCategoryClick()
                    }else {
                        MaterialAlertDialogBuilder(container.context)
                        .setView(R.layout.dialog_category_limit)
                        .setPositiveButton("Rozumím", null)
                        .show()
                    }
                }
                container.addView(addView)

            } else {
                title.text = category.name
                container.visibility = if (category.isExpanded) View.VISIBLE else View.GONE

                if (category.isExpanded) {
                    val folderCount = category.folders.count { it.key != "[ADD_NEW_FOLDER]" }
                    category.folders.forEach { (folderName, files) ->
                        val view = if (folderName == "[ADD_NEW_FOLDER]") {
                            createAddButton(R.layout.add_folder_button) {
                                if (folderCount < 7){
                                    onAddFolderClick(category.name)
                                }else {
                                    MaterialAlertDialogBuilder(container.context)
                                        .setView(R.layout.dialog_folder_limit)
                                        .setPositiveButton("Rozumím", null)
                                        .show()
                                }
                            }
                        } else {
                            createFolderView(category.name, folderName, files, category.expandedFolderName)
                        }
                        container.addView(view)
                    }

                }
                itemView.setOnClickListener { onCategoryClick(category.name) }
                itemView.setOnLongClickListener {
                    val options = arrayOf("Přejmenovat Kategorii", "Smazat celou kategorii")

                    MaterialAlertDialogBuilder(container.context)
                        .setTitle("Kategorie ${category.name}")
                        .setItems(options) { _, which ->
                            when (which) {
                                0 -> onCategLongClickRename(category.name)
                                1 -> onCategLongClickDelete(category.name)
                            }
                        }
                        .show()
                    true
                }
            }
        }
        private fun createAddButton(layoutId: Int, onClick: () -> Unit): View {
            val view = LayoutInflater.from(container.context).inflate(layoutId, container, false)
            view.setOnClickListener { onClick() }
            return view
        }
        private fun createFolderView(
            categoryName: String,
            folderName: String,
            files: List<SdFile>,
            expandedFolderName: String?
        ): View {
            val folderRow = LayoutInflater.from(container.context).inflate(R.layout.item_folder, container, false)
            val folderNameTxt = folderRow.findViewById<TextView>(R.id.folderName)
            val filesListContainer = folderRow.findViewById<LinearLayout>(R.id.filesListContainer)

            folderNameTxt.text = folderName

            val isFolderExpanded = folderName == expandedFolderName
            filesListContainer.visibility = if (isFolderExpanded) View.VISIBLE else View.GONE

            folderRow.setOnClickListener {
                onFolderClick(folderName)
            }
            folderRow.setOnLongClickListener {
                val options = arrayOf("Přejmenovat složku", "Smazat celou složku")

                MaterialAlertDialogBuilder(container.context)
                    .setTitle("Složka $folderName")
                    .setItems(options) { _, which ->
                        val fullPath = "${categoryName}_$folderName"
                        when (which) {
                            0 -> onFolderLongClickRename(categoryName, folderName, fullPath)
                            1 -> onFolderLongClickDelete(fullPath)
                        }
                    }
                    .show()
                true
            }
            if (isFolderExpanded) {
                files.forEach { file ->
                    val fileView = createFileView(file, files)
                    filesListContainer.addView(fileView)
                }
            }
            return folderRow
        }
        private fun createFileView(file: SdFile, parentFolderFiles: List<SdFile>): View {
            val inflater = LayoutInflater.from(itemView.context)
            val actualFilesCount = parentFolderFiles.size - 1
            return if (file.fileName == "[ADD_NEW_FILE]") {
                val addView = inflater.inflate(R.layout.add_file_button, container, false)
                addView.setOnClickListener {
                    if (actualFilesCount < 5) {
                        onAddFileClick()
                    } else {
                        MaterialAlertDialogBuilder(container.context)
                            .setView(R.layout.dialog_file_limit)
                            .setPositiveButton("Rozumím", null)
                            .show()
                    }
                }
                addView
            } else {
                val fileView = inflater.inflate(R.layout.item_file, container, false)
                val fileNameTxt = fileView.findViewById<TextView>(R.id.fileName)

                fileNameTxt.text = file.fileName

                fileView.setOnLongClickListener {
                    val options = arrayOf("Přejmenovat sloubor", "Smazat soubor")

                    MaterialAlertDialogBuilder(container.context)
                        .setTitle("Soubor ${file.fileName}")
                        .setItems(options) { _, which ->
                            when (which) {
                                0 -> onFileRename(file)
                                1 -> onFileDelete(file)
                            }
                        }
                        .show()
                    true
                }
                fileView
            }
        }
    }
    fun clearData() {
        submitList(emptyList())
    }
}

class DiffCallback : androidx.recyclerview.widget.DiffUtil.ItemCallback<CategoryUiModel>() {
    override fun areItemsTheSame(oldItem: CategoryUiModel, newItem: CategoryUiModel): Boolean {
        return oldItem.name == newItem.name
    }

    override fun areContentsTheSame(oldItem: CategoryUiModel, newItem: CategoryUiModel): Boolean {
        return oldItem == newItem
    }
}