package com.example.toysender

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.TextView
import androidx.core.widget.NestedScrollView
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import java.text.SimpleDateFormat
import java.util.*

class RemoteFragment: Fragment() {
    private lateinit var onoffButton: Button
    private lateinit var volUpButton: Button
    private lateinit var volDownButton: Button
    private lateinit var changeCategoryButton: Button
    private lateinit var changeGameButton: Button
    private lateinit var debugTextView: TextView
    private lateinit var debugScroll: NestedScrollView
    private val viewModel: MyViewModel by activityViewModels()
    private fun getActiveManager() = viewModel.getActiveManager()

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        val view = inflater.inflate(R.layout.remote_fragment, container, false)

        onoffButton = view.findViewById(R.id.onoffButton)
        onoffButton.setOnClickListener { getActiveManager().toggleToy() }

        volUpButton = view.findViewById(R.id.volumeUpButton)
        volUpButton.setOnClickListener {
            getActiveManager().sendData(byteArrayOf(MyBleManager.MT_VOL_UP))
        }

        volDownButton = view.findViewById(R.id.volumeDownButton)
        volDownButton.setOnClickListener {
            getActiveManager().sendData(byteArrayOf(MyBleManager.MT_VOL_DOWN))
        }

        changeGameButton = view.findViewById(R.id.changeGameButton)
        changeGameButton.setOnClickListener {
            getActiveManager().sendData(byteArrayOf(MyBleManager.MT_G_MODE_CHANGE))
        }

        changeCategoryButton = view.findViewById(R.id.changeCategoryButton)
        changeCategoryButton.setOnClickListener {
            getActiveManager().sendData(byteArrayOf(MyBleManager.MT_CHANGE_CATEGORY))
        }

        debugTextView = view.findViewById(R.id.debugTextView)
        debugScroll = view.findViewById(R.id.debugScroll)

        return view
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        viewModel.activeToyIndex.observe(viewLifecycleOwner) { _ ->
            debugTextView.text = ""

            val activeMgr = getActiveManager()

            activeMgr.debugData.removeObservers(viewLifecycleOwner)
            activeMgr.debugData.observe(viewLifecycleOwner) { info ->
                val currentText = debugTextView.text.toString()
                debugTextView.text = "$info\n----------------\n$currentText".take(1500)
                debugScroll.fullScroll(View.FOCUS_UP)
            }
            activeMgr.batteryInfo.removeObservers(viewLifecycleOwner)
            activeMgr.batteryInfo.observe(viewLifecycleOwner) { info ->
                updateOnOffButton(info.isDeviceOn)
            }
        }
    }
    private fun updateOnOffButton(isOn: Boolean) {
        if (isOn) {
            onoffButton.stateDescription = "Vypnout hračku"
            onoffButton.backgroundTintList = android.content.res.ColorStateList.valueOf(android.graphics.Color.parseColor("#4CAF50"))
        } else {
            onoffButton.stateDescription = "Zapnout hračku"
            onoffButton.backgroundTintList = android.content.res.ColorStateList.valueOf(android.graphics.Color.parseColor("#F44336"))
        }
    }
}