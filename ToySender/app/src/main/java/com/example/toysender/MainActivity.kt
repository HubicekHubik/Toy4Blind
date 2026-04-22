package com.example.toysender

import android.bluetooth.BluetoothDevice
import android.os.Bundle
import androidx.activity.viewModels
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.setupWithNavController
import com.google.android.material.bottomnavigation.BottomNavigationView
import kotlinx.coroutines.launch
import kotlinx.coroutines.delay
import no.nordicsemi.android.ble.observer.ConnectionObserver
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

class MainActivity : AppCompatActivity() {
    private val viewModel: MyViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val topAppBar = findViewById<com.google.android.material.appbar.MaterialToolbar>(R.id.topAppBar)
        val bottomNav = findViewById<BottomNavigationView>(R.id.bottom_navigation)

        setupBleCallbacks()

        val navHostFragment = supportFragmentManager
            .findFragmentById(R.id.fragment_container) as NavHostFragment
        val navController = navHostFragment.navController
        bottomNav.setupWithNavController(navController)

        bottomNav.setOnItemSelectedListener { item ->
            when (item.itemId) {
                R.id.nav_swaptoy -> {
                    viewModel.swapToys()
                    true
                }
                else -> {
                    androidx.navigation.ui.NavigationUI.onNavDestinationSelected(item, navController)
                    true
                }
            }
        }
        topAppBar.post {
            val batteryItem = topAppBar.menu.findItem(R.id.action_battery)
            val actionView = batteryItem?.actionView

            if (actionView != null) {
                val batteryIcon = actionView.findViewById<android.widget.ImageView>(R.id.batteryIcon)
                val batteryText = actionView.findViewById<android.widget.TextView>(R.id.batteryText)

                viewModel.activeToyIndex.observe(this) { activeIdx ->
                    val activeManager = viewModel.getActiveManager()

                    activeManager.batteryInfo.removeObservers(this)
                    activeManager.batteryInfo.observe(this) { info ->
                        updateBatteryUI(info, batteryIcon, batteryText)
                    }
                }
            }
        }

        viewModel.activeToyIndex.observe(this) { index ->
            val stateLiveData = viewModel.getActiveConnectionState()

            stateLiveData.removeObservers(this)
            stateLiveData.observe(this) { state ->
                val isConnected = state == BleState.CONNECTED
                bottomNav.menu.findItem(R.id.nav_remote).isEnabled = isConnected

                if (!isConnected && navController.currentDestination?.id == R.id.nav_remote) {
                    navController.navigate(R.id.nav_library)
                }
            }
        }

        setupConnectionObservers()

        fun showInfoDialog() {
            com.google.android.material.dialog.MaterialAlertDialogBuilder(this)
                .setTitle("Jak funguje ToySender")
                .setMessage(
                    "1. Po kliknutí na tlačítko připojit se aplikace automaticky připojí k hračce.\n\n" +
                    "2. Po úspěšném připojení se zobrazí audio obsah.\n\n" +
                    "3. V prostředí knihovny je možné přidávat nové kategorie, upravovat a nahrávat soubory(zvuky)\n\n" +
                            "\t\tmaximální počet kategorii je 8\n"+
                            "\t\tmaximální počet druhů je 5\n"+
                            "\t\tmaximální počet zvuků je 5\n\n"+
                    "4. Nahrávejte soubory, které mají maximálně 3s, jinak není zaručen správný chod hračky\n\n"+
                    "5. Sekce ovladač se zpřístupní po připojení, uvnitř lze měnit hlasitost, přepínat kategorie a herní režimy\n"
                )
                .setPositiveButton("Rozumím", null)
                .show()
        }

        topAppBar.setOnMenuItemClickListener { menuItem ->
            when (menuItem.itemId) {
                R.id.action_info -> {
                    showInfoDialog()
                    true
                }
                else -> false
            }
        }

    }
    private fun setupBleCallbacks() {
        viewModel.bleManager1.onPathReceived = { path ->
            if (path == "CLEAR_ALL_FILES_SIGNAL") viewModel.clearToyData(0)
            else viewModel.parseAndAddPath(0, path)
        }

        viewModel.bleManager2.onPathReceived = { path ->
            if (path == "CLEAR_ALL_FILES_SIGNAL") viewModel.clearToyData(1)
            else viewModel.parseAndAddPath(1, path)
        }
    }
    private fun setupConnectionObservers() {
        viewModel.bleManager1.connectionObserver = object : ConnectionObserver {
            override fun onDeviceConnecting(device: BluetoothDevice) {}
            override fun onDeviceConnected(device: BluetoothDevice) {
                viewModel.connectionState1.postValue(BleState.CONNECTED)
            }
            override fun onDeviceFailedToConnect(device: BluetoothDevice, reason: Int) {
                viewModel.connectionState1.postValue(BleState.DISCONNECTED)
            }
            override fun onDeviceReady(device: BluetoothDevice) {
                viewModel.bleManager1.sendData(byteArrayOf(MyBleManager.MT_REQ_LASTBAT))
            }
            override fun onDeviceDisconnecting(device: BluetoothDevice) {}
            override fun onDeviceDisconnected(device: BluetoothDevice, reason: Int) {
                viewModel.connectionState1.postValue(BleState.DISCONNECTED)
                viewModel.clearToyData(0)

                lifecycleScope.launch {
                    viewModel.setScanEnabled(false)
                    delay(3.seconds)
                    viewModel.setScanEnabled(true)
                }
            }
        }

        viewModel.bleManager2.connectionObserver = object : ConnectionObserver {
            override fun onDeviceConnecting(device: BluetoothDevice) {}
            override fun onDeviceConnected(device: BluetoothDevice) {
                viewModel.connectionState2.postValue(BleState.CONNECTED)
            }
            override fun onDeviceFailedToConnect(device: BluetoothDevice, reason: Int) {
                viewModel.connectionState1.postValue(BleState.DISCONNECTED)
            }
            override fun onDeviceReady(device: BluetoothDevice) {
                viewModel.bleManager2.sendData(byteArrayOf(MyBleManager.MT_REQ_LASTBAT))
            }
            override fun onDeviceDisconnecting(device: BluetoothDevice) {}
            override fun onDeviceDisconnected(device: BluetoothDevice, reason: Int) {
                viewModel.connectionState2.postValue(BleState.DISCONNECTED)
                viewModel.clearToyData(1)

                lifecycleScope.launch {
                    viewModel.setScanEnabled(false)
                    delay(3.seconds)
                    viewModel.setScanEnabled(true)
                }
            }
        }
    }
    private fun getThemeColor(attr: Int): Int {
        val typedValue = android.util.TypedValue()
        theme.resolveAttribute(attr, typedValue, true)
        return typedValue.data
    }

    private fun updateBatteryUI(info: BatteryInfo, icon: android.widget.ImageView, text: android.widget.TextView) {
        val pct = mvToPercent(info.voltageMv)
        text.text = "$pct%"
        icon.setImageResource(R.drawable.battery_status_indicator)
        val finalLevel = if (info.isCharging) pct + 100 else pct
        icon.setImageLevel(finalLevel)

        val color = when {
            info.isCharging -> getThemeColor(com.google.android.material.R.attr.colorTertiary)
            pct <= 15 -> getThemeColor(com.google.android.material.R.attr.colorError)
            else -> getThemeColor(com.google.android.material.R.attr.colorOnSurfaceVariant)
        }
        icon.imageTintList = android.content.res.ColorStateList.valueOf(color)
        text.setTextColor(color)
    }
    private fun mvToPercent(mv: Int): Int {
        val estimatedBatteryMv = mv + 200

        return when {
            estimatedBatteryMv >= 4100 -> 100
            estimatedBatteryMv >= 3950 -> 90
            estimatedBatteryMv >= 3850 -> 75
            estimatedBatteryMv >= 3750 -> 50
            estimatedBatteryMv >= 3650 -> 25
            estimatedBatteryMv >= 3550 -> 10
            estimatedBatteryMv >= 3450 -> 5
            else -> 0
        }
    }
}