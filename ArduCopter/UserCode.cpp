#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // CH7: Observer 制御ON/OFF (3ポジションスイッチ)
    // HIGH=ON, MIDDLE/LOW=OFF
    const bool enable = (ch_flag == RC_Channel::AuxSwitchPos::HIGH);
    observer.set_control_enabled(enable);
#if HAL_GCS_ENABLED
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: Control %s", enable ? "ENABLED" : "DISABLED");
#endif
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // CH8: EKFリセット (モーメンタリスイッチ: HIGH→LOWエッジ検出)
    static RC_Channel::AuxSwitchPos prev = RC_Channel::AuxSwitchPos::LOW;
    if (prev == RC_Channel::AuxSwitchPos::HIGH && ch_flag == RC_Channel::AuxSwitchPos::LOW) {
        observer.reset_ekf_to_initial_state();
    }
    prev = ch_flag;
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
