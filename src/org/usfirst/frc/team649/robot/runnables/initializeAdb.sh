#!/bin/bash
adb devices
adb connect 10.6.49.89:5555
adb shell input keyevent 82
adb shell monkey -p com.example.suneelbelkhale.vision649 -c android.intent.category.LAUNCHER 1
