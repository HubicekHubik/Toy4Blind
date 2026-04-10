package com.example.toysender

import android.os.Bundle
import androidx.activity.viewModels
import androidx.appcompat.app.AppCompatActivity
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.setupWithNavController
import com.google.android.material.bottomnavigation.BottomNavigationView

class MainActivity : AppCompatActivity() {
    lateinit var bleManager: MyBleManager
    private val viewModel: MyViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        val topAppBar = findViewById<com.google.android.material.appbar.MaterialToolbar>(R.id.topAppBar)

        bleManager = MyBleManager(this)

        val navHostFragment = supportFragmentManager
            .findFragmentById(R.id.fragment_container) as NavHostFragment
        val navController = navHostFragment.navController
        val bottomNav = findViewById<BottomNavigationView>(R.id.bottom_navigation)
        bottomNav.setupWithNavController(navController)

        topAppBar.post {
            val batteryItem = topAppBar.menu.findItem(R.id.action_battery)
            val actionView = batteryItem?.actionView

            if (actionView != null) {
                val batteryIcon = actionView.findViewById<android.widget.ImageView>(R.id.batteryIcon)
                val batteryText = actionView.findViewById<android.widget.TextView>(R.id.batteryText)

                // Sledujeme změny baterie z bleManageru
                bleManager.batteryInfo.observe(this) { info ->
                    val pct = mvToPercent(info.voltageMv)
                    batteryText.text = "$pct%"

                    // Nastavení LevelListu (obrázek, co se mění podle levelu)
                    batteryIcon.setImageResource(R.drawable.battery_status_indicator)

                    // Výpočet levelu (0-100 vybíjení, 101-200 nabíjení)
                    val finalLevel = if (info.isCharging) pct + 100 else pct
                    batteryIcon.setImageLevel(finalLevel)

                    // Dynamická změna barvy podle stavu (Material 3)
                    val color = when {
                        info.isCharging -> getThemeColor(com.google.android.material.R.attr.colorTertiary)
                        pct <= 15 -> getThemeColor(com.google.android.material.R.attr.colorError)
                        else -> getThemeColor(com.google.android.material.R.attr.colorOnSurfaceVariant)
                    }
                    batteryIcon.imageTintList = android.content.res.ColorStateList.valueOf(color)
                    batteryText.setTextColor(color)
                }
            }
        }

        viewModel.connectionState.observe(this) { state ->
            val isConnected = state == BleState.CONNECTED

            bottomNav.menu.findItem(R.id.nav_remote).isEnabled = isConnected

            if (!isConnected && navController.currentDestination?.id == R.id.nav_remote) {
                navController.navigate(R.id.nav_library)
            }
        }
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
    private fun getThemeColor(attr: Int): Int {
        val typedValue = android.util.TypedValue()
        theme.resolveAttribute(attr, typedValue, true)
        return typedValue.data
    }

    private fun mvToPercent(mv: Int): Int {
        return when {
            mv >= 4100 -> 100
            mv >= 3950 -> 90
            mv >= 3850 -> 75
            mv >= 3750 -> 50
            mv >= 3650 -> 25
            mv >= 3550 -> 10
            else -> 0
        }
    }
}