/*
  xnrg_01_hlw8012.ino - HLW8012 (Sonoff Pow) energy sensor support for Tasmota

  Copyright (C) 2020  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_ENERGY_SENSOR
#ifdef USE_RESISTIVE_LOAD
/*********************************************************************************************\
 * Resistive load on the outputs
\*********************************************************************************************/

#define XNRG_90                90

struct RESLOAD {
  unsigned long energy_period_counter = 0;

  unsigned long power_load[2] = { 400, 1250 };   /* W */

  unsigned long currentRelayStates;
} ResLoad;

/********************************************************************************************/

void ResLoadEvery200ms(void)
{
  uint8_t i;
  float tmpActivePower = 0.0;

  for (i = 0; i < 2; i++) {
    if (bitRead(ResLoad.currentRelayStates, i))
    {
      tmpActivePower += ResLoad.power_load[i];
    }
  }  
  Energy.active_power[0] = tmpActivePower;
  Energy.kWhtoday_delta += (tmpActivePower / 3600.0 / 5.0) * 1e5; 
}

void ResLoadEverySecond(void)
{
  EnergyUpdateToday();
}

void ResLoadSnsInit(void)
{

}

void ResLoadDrvInit(void)
{
  Energy.current_available = false;
  Energy.voltage_available = false;
  energy_flg = XNRG_90;
}

bool ResLoadCommand(void)
{
  bool serviced = true;

  if ((CMND_POWERCAL == Energy.command_code) || (CMND_VOLTAGECAL == Energy.command_code) || (CMND_CURRENTCAL == Energy.command_code)) {
    // Service in xdrv_03_energy.ino
  }
  else if (CMND_POWERSET == Energy.command_code) {
    if (XdrvMailbox.data_len && Hlw.cf_power_pulse_length ) {
      Settings.energy_power_calibration = ((unsigned long)(CharToFloat(XdrvMailbox.data) * 10) * Hlw.cf_power_pulse_length ) / Hlw.power_ratio;
    }
  }
  else if (CMND_VOLTAGESET == Energy.command_code) {
    if (XdrvMailbox.data_len && Hlw.cf1_voltage_pulse_length ) {
      Settings.energy_voltage_calibration = ((unsigned long)(CharToFloat(XdrvMailbox.data) * 10) * Hlw.cf1_voltage_pulse_length ) / Hlw.voltage_ratio;
    }
  }
  else if (CMND_CURRENTSET == Energy.command_code) {
    if (XdrvMailbox.data_len && Hlw.cf1_current_pulse_length) {
      Settings.energy_current_calibration = ((unsigned long)(CharToFloat(XdrvMailbox.data)) * Hlw.cf1_current_pulse_length) / Hlw.current_ratio;
    }
  }
  else serviced = false;  // Unknown command

  return serviced;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xnrg90(uint8_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_EVERY_200_MSECOND:
      ResLoadEvery200ms();
      break;
    case FUNC_ENERGY_EVERY_SECOND:
      ResLoadEverySecond();
      break;
    case FUNC_SET_POWER:
      /* getRelayState */
      ResLoad.currentRelayStates = XdrvMailbox.index;
      AddLog_P2(LOG_LEVEL_INFO, PSTR("RLOAD: RelayStatus: %d"), ResLoad.currentRelayStates);
      break;
    case FUNC_COMMAND:
      result = ResLoadCommand();
      break;
    case FUNC_INIT:
      ResLoadSnsInit();
      break;
    case FUNC_PRE_INIT:
      ResLoadDrvInit();
      break;
  }
  return result;
}

#endif  // USE_RESISTIVE_LOAD
#endif  // USE_ENERGY_SENSOR
