#pragma once

namespace buzzer {

enum class Cue {
  Startup,
  RemoteConnected,
};

bool begin();
bool play(Cue cue);
void setMandatoryLowBatteryAlarm(bool active);
bool alarmActive();

}  // namespace buzzer
