package com.example.toysender

import kotlin.getValue

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.graphics.Color
import android.net.Uri
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.LinearLayout
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import no.nordicsemi.android.ble.observer.ConnectionObserver
import android.view.accessibility.AccessibilityManager
class nav_fragment : Fragment() {
    private lateinit var scanButton: Button
    private var scanner: BluetoothLeScanner? = null
    private lateinit var bleManager: MyBleManager
    private val viewModel: MyViewModel by activityViewModels()
    private var selectedFileUri: Uri? = null
    // Register the file picker launcher
    private val filePickerLauncher = registerForActivityResult(
        ActivityResultContracts.GetContent()
    ) { uri: Uri? ->
        if (uri != null) {
            selectedFileUri = uri
            MaterialAlertDialogBuilder(requireContext())
                .setView(R.layout.dialog_submit_file)
                .setNegativeButton("Zrušit", null)
                .setPositiveButton("Odeslat"){_, _ ->
                    fileSubmit()
                }
            .show()
        }
    }
    private lateinit var recyclerView: RecyclerView

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
                          savedInstanceState: Bundle?
    ): View? {

        val view = inflater.inflate(R.layout.nav_fragment, container, false)

        recyclerView = view.findViewById(R.id.recyclerView)

        scanButton = view.findViewById(R.id.scanButton)

        bleManager = (requireActivity() as MainActivity).bleManager

        bleManager.onPathReceived = { path ->
            if (path == "CLEAR_ALL_FILES_SIGNAL") {
                viewModel.clearAllData()
            } else {
                viewModel.parseAndAddPath(path)
            }
        }

        val adapter = CategoryAdapter(
            onCategoryClick = { name -> viewModel.toggleCategory(categName = name)},
            onFolderClick = { folder -> viewModel.toggleFolder(folder) },

            onFolderLongClickDelete = { path -> showDeleteConfirmDialog(path, MyBleManager.MT_DELETE_SD_FILE)},
            onFolderLongClickRename = { categoryName, oldFolderName, fullPath -> showRenameFolderDialog(categoryName, oldFolderName, fullPath,MyBleManager.MT_RENAME_SD_FOLDER)},

            onCategLongClickDelete = { category -> showDeleteConfirmDialog(category, MyBleManager.MT_DELETE_SD_CATEGORY)},
            onCategLongClickRename = { category -> showRenameFolderDialog(category,"", "", MyBleManager.MT_RENAME_SD_CATEGORY)},

            onFileDelete = { file -> bleManager.requestDelete(file.fullPath, MyBleManager.MT_DELETE_SD_FILE) },
            onFileRename = { file -> showRenameDialog(file)},

            onAddFileClick = { filePickerLauncher.launch("audio/x-wav") },
            onAddFolderClick = { categoryName -> showCreateFolderDialog(categoryName) },
            onAddCategoryClick = { showCreateCategoryDialog() },

        )
        recyclerView.adapter = adapter
        recyclerView.layoutManager = LinearLayoutManager(requireContext())

        if (isTalkBackEnabled()) {
            recyclerView.itemAnimator = null
        }

        fun updateUiList() {
            val currentMap = viewModel.groupedFiles.value ?: emptyMap()
            val expCat = viewModel.expandedCategory.value
            val expFold = viewModel.expandedFolder.value

            val uiModels = currentMap.map { (categoryName, foldersMap) ->

                val foldersWithAddFile = foldersMap.mapValues { (folderName, files) ->
                    files + SdFile(categoryName, folderName, "[ADD_NEW_FILE]", "INTERNAL_ACTION")
                }

                val finalFolders = foldersWithAddFile + ("[ADD_NEW_FOLDER]" to emptyList<SdFile>())

                CategoryUiModel(
                    name = categoryName,
                    folders = finalFolders,
                    isExpanded = categoryName == expCat,
                    expandedFolderName = expFold
                )
            }

            val finalList = if (viewModel.connectionState.value == BleState.CONNECTED) {
                uiModels + CategoryUiModel(
                    name = "[ADD_NEW_CATEGORY]",
                    folders = emptyMap(),
                    isExpanded = false,
                    expandedFolderName = null
                )
            } else {
                emptyList()
            }
            adapter.submitList(finalList.toList())
        }

        viewModel.groupedFiles.observe(viewLifecycleOwner) { map ->
            updateUiList()
        }

        viewModel.expandedFolder.observe(viewLifecycleOwner) { expandedFolder ->
            updateUiList()
        }

        viewModel.expandedCategory.observe(viewLifecycleOwner) { expanded ->
            updateUiList()
        }

        viewModel.connectionState.observe(viewLifecycleOwner) { state ->
            when (state) {
                BleState.SCANNING -> {
                }
                BleState.CONNECTED -> {
                    updateScanButton(isConnected = true)
                }
                BleState.DISCONNECTED -> {
                    updateScanButton(isConnected = false)
                    scanButton.clearAnimation()
                    viewModel.expandedCategory.value = null
                    viewModel.expandedFolder.value = null
                    viewModel.clearAllData()
                }
                else -> {}
            }
        }

        bleManager.connectionObserver = object : ConnectionObserver {
            override fun onDeviceConnecting(device: BluetoothDevice) {
            }

            override fun onDeviceConnected(device: BluetoothDevice) {
                viewModel.connectionState.postValue(BleState.CONNECTED)
            }

            override fun onDeviceFailedToConnect(device: BluetoothDevice, reason: Int) {
            }

            override fun onDeviceReady(device: BluetoothDevice) {
            }

            override fun onDeviceDisconnecting(device: BluetoothDevice) {
            }

            override fun onDeviceDisconnected(device: BluetoothDevice, reason: Int) {
                viewModel.connectionState.postValue(BleState.DISCONNECTED)
            }
        }

        scanButton.setOnClickListener {
            if (viewModel.connectionState.value == BleState.CONNECTED) {
                bleManager.disconnect().enqueue()
                viewModel.connectionState.value = BleState.DISCONNECTED
                return@setOnClickListener
            }
            viewModel.connectionState.value = BleState.SCANNING
            startBleScan()
        }
        return view
    }

    @SuppressLint("MissingPermission")
    private fun startBleScan() {
        val bluetoothManager = requireContext().getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val adapter = bluetoothManager.adapter
        scanner = adapter?.bluetoothLeScanner

        if (scanner == null) {
            return
        }

        val permissions = arrayOf(
            Manifest.permission.BLUETOOTH_SCAN,
            Manifest.permission.BLUETOOTH_CONNECT,
            Manifest.permission.ACCESS_FINE_LOCATION
        )

        val missingPermissions = permissions.filter {
            ContextCompat.checkSelfPermission(requireContext(), it) != PackageManager.PERMISSION_GRANTED
        }

        if (missingPermissions.isNotEmpty()) {
            return
        }

        scanner?.startScan(leScanCallback)
    }

    @SuppressLint("MissingPermission")
    private fun stopBleScan() {
        scanner?.stopScan(leScanCallback)
        scanner = null
    }

    private val leScanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            val name = device.name ?: "Neznámé"

            if (name.contains("Toy", ignoreCase = true) || name.contains("Nordic", ignoreCase = true)) {
                stopBleScan()
                connectToDevice(device)
            }
        }

        override fun onScanFailed(errorCode: Int) {
        }
    }
    private fun showCreateCategoryDialog(){
        val dialogView = layoutInflater.inflate(R.layout.dialog_edit, null)
        val editText = dialogView.findViewById<com.google.android.material.textfield.TextInputEditText>(R.id.editText)

        MaterialAlertDialogBuilder(requireContext())
            .setTitle("Nová Kategorie")
            .setMessage("Vytvořit novou kategori")
            .setView(dialogView)
            .setPositiveButton("Vytvořit") { _, _ ->
                val categName = editText.text.toString().trim()
                if (categName.isNotEmpty() && categName != "Q") {
                    viewModel.parseAndAddPath("${categName}/")
                }
            }
            .setNegativeButton("Zrušit", null)
            .show()
    }
    private fun showCreateFolderDialog(categoryName: String) {
        val dialogView = layoutInflater.inflate(R.layout.dialog_edit, null)
        val editText = dialogView.findViewById<com.google.android.material.textfield.TextInputEditText>(R.id.editText)

        MaterialAlertDialogBuilder(requireContext())
            .setTitle("Nová složka")
            .setMessage("Vytvořit novou složku v sekci $categoryName")
            .setView(dialogView)
            .setPositiveButton("Vytvořit") { _, _ ->
                val folderName = editText.text.toString().trim()
                if (folderName.isNotEmpty()) {
                    val dirPath = categoryName + '_' + folderName
                    bleManager.requestAdd(dirPath, MyBleManager.MT_ADD_DIR)
                }
            }
            .setNegativeButton("Zrušit", null)
            .show()
    }

    private fun showDeleteConfirmDialog(fullPath: String, cmd: Byte) {
        MaterialAlertDialogBuilder(requireContext())
            .setTitle("Odstranit")
            .setMessage("Opravdu chcete odstranit $fullPath?")
            .setPositiveButton("Odstranit") { _, _ ->
                viewModel.clearAllData()
                bleManager.requestDelete(fullPath, cmd)
            }
            .setNegativeButton("Zrušit", null)
            .show()
    }

    private fun showRenameFolderDialog(categoryName: String, oldFolderName: String, fullPath: String, cmd: Byte) {
        val dialogView = layoutInflater.inflate(R.layout.dialog_edit, null)
        val editText = dialogView.findViewById<com.google.android.material.textfield.TextInputEditText>(R.id.editText)
        if (cmd != MyBleManager.MT_RENAME_SD_CATEGORY){
            editText.setText(oldFolderName)
            MaterialAlertDialogBuilder(requireContext())
                .setTitle("Přejmenovat složku")
                .setMessage("Přejmenovat $oldFolderName na:")
                .setView(dialogView)
                .setPositiveButton("Přejmenovat") { _, _ ->
                    val newName = editText.text.toString().trim()
                    if (newName.isNotEmpty() && newName != oldFolderName) {
                        val finalDir = "${categoryName}_$newName"
                        bleManager.requestRename(fullPath, finalDir, cmd)
                    }
                }
                .setNegativeButton("Zrušit", null)
                .show()
        }else {
            editText.setText(categoryName)
            MaterialAlertDialogBuilder(requireContext())
                .setTitle("Přejmenovat Kategorii")
                .setMessage("Přejmenovat $categoryName na:")
                .setView(dialogView)
                .setPositiveButton("Přejmenovat") { _, _ ->
                    val newName = editText.text.toString().trim()
                    if (newName == "Q"){

                    }
                    if (newName.isNotEmpty() && newName != categoryName) {
                        bleManager.requestRename(categoryName, newName, cmd)
                    }
                }
                .setNegativeButton("Zrušit", null)
                .show()
        }

    }
    private fun showRenameDialog(file: SdFile) {
        val dialogView = layoutInflater.inflate(R.layout.dialog_edit, null)
        val editText = dialogView.findViewById<com.google.android.material.textfield.TextInputEditText>(R.id.editText)

        MaterialAlertDialogBuilder(requireContext())
            .setTitle("Přejmenovat")
            .setMessage("Chystáte se přejmenovat ${file.fileName}")
            .setView(dialogView)
            .setPositiveButton("Přejmentovat") { _, _ ->
                var newName = editText.text.toString().trim()
                if (!newName.lowercase().endsWith(".wav")) {
                    newName += ".wav"
                }
                if (newName.isNotEmpty()) {
                    bleManager.requestRename(file.fullPath ,newName ,MyBleManager.MT_RENAME_SD_FILE)
                }
            }
            .setNegativeButton("Zrušit", null)
            .show()
    }
    private fun connectToDevice(device: BluetoothDevice) {
        bleManager.connect(device)
            .retry(3, 100)
            .useAutoConnect(false)
            .enqueue()
    }
    private fun updateScanButton(isConnected: Boolean) {
        val density = resources.displayMetrics.density
        val params = scanButton.layoutParams as LinearLayout.LayoutParams

        if (isConnected) {
            scanButton.text = getString(R.string.status_connected)
            scanButton.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#4CAF50"))
            params.width = (150 * density).toInt()
            params.height = (100 * density).toInt()
        } else {
            scanButton.text = getString(R.string.status_scanning)
            scanButton.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#2E55A2"))
            params.width = (150 * density).toInt()
            params.height = (150 * density).toInt()
        }
        scanButton.layoutParams = params
    }

    fun fileSubmit() {
        val category = viewModel.expandedCategory.value
        val folder = viewModel.expandedFolder.value

        val currentDir = "${category}_${folder}"

        if (selectedFileUri != null && category != null && folder != null) {
            bleManager.startSendingFile(selectedFileUri, currentDir)
        }
    }
    private fun isTalkBackEnabled(): Boolean {
        val am = requireContext().getSystemService(Context.ACCESSIBILITY_SERVICE) as AccessibilityManager

        return am.isEnabled && am.isTouchExplorationEnabled
    }
}