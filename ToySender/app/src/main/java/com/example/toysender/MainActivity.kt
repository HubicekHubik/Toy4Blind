package com.example.toysender

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.setupWithNavController
import com.google.android.material.bottomnavigation.BottomNavigationView

class MainActivity : AppCompatActivity() {
    lateinit var bleManager: MyBleManager

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

        fun showInfoDialog() {
            com.google.android.material.dialog.MaterialAlertDialogBuilder(this)
                .setTitle("Jak funguje ToySender")
                .setMessage(
                    "1. Po kliknutí na tlačítko připojit se aplikace automaticky připojí k hračce.\n\n" +
                    "2. Po úspěšném připojení se zobrazí audio obsah.\n\n" +
                    "3. V prostředí knihovny je možné přidávat nové kategorie, upravovat a nahrávat soubory\n\n" +
                            "\t\tmaximální počet kategorii je 8\n"+
                            "\t\tmaximální počet druhů je 5\n"+
                            "\t\tmaximální počet zvuků je 5\n\n"+
                    "4. Nahrávejte soubory, které mají maximálně 3s, jinak není zaručen správný chod hračky\n\n"+
                    "5. V sekci ovladač, lze měnit hlasitost, přepínat kategorie a herní režimy\n"
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
}