#pragma once

// Powers the Pi down through systemd (`systemctl poweroff`; the service user
// needs systemd/50-dlidar-poweroff.rules). False if it was refused.
bool PowerOff();
