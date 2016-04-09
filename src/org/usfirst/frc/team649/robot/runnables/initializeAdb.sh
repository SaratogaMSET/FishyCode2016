#!/bin/bash
#adb kill-server
#adb start-server
adb reverse tcp:5805 tcp:5805
adb shell input keyevent 82
adb shell monkey -p com.example.suneelbelkhale.vision649 -c android.intent.category.LAUNCHER 1
