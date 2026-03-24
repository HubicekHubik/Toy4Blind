package com.example.toysender

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.TextView
import androidx.core.widget.NestedScrollView
import androidx.fragment.app.Fragment
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

    private lateinit var bleManager: MyBleManager

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        val view = inflater.inflate(R.layout.remote_fragment, container, false)

        bleManager = (requireActivity() as MainActivity).bleManager

        onoffButton = view.findViewById(R.id.onoffButton)
        onoffButton.setOnClickListener { bleManager.toggleToy() }

        volUpButton = view.findViewById(R.id.volumeUpButton)
        volUpButton.setOnClickListener {
            bleManager.sendData(byteArrayOf(MyBleManager.MT_VOL_UP))
        }

        volDownButton = view.findViewById(R.id.volumeDownButton)
        volDownButton.setOnClickListener {
            bleManager.sendData(byteArrayOf(MyBleManager.MT_VOL_DOWN))
        }

        changeGameButton = view.findViewById(R.id.changeGameButton)
        changeGameButton.setOnClickListener {
            bleManager.sendData(byteArrayOf(MyBleManager.MSG_TYPE_TOY_SWITCH))
        }

        changeCategoryButton = view.findViewById(R.id.changeCategoryButton)
        changeCategoryButton.setOnClickListener {
            bleManager.sendData(byteArrayOf(MyBleManager.MT_CHANGE_CATEGORY))
        }

        debugTextView = view.findViewById(R.id.debugTextView)
        debugScroll = view.findViewById(R.id.debugScroll)

        return view
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        bleManager.debugData.observe(viewLifecycleOwner) { info ->
            val currentText = debugTextView.text.toString()

            debugTextView.text = "$info\n----------------\n$currentText".take(1500)

            debugScroll.fullScroll(View.FOCUS_UP)
        }
    }
}