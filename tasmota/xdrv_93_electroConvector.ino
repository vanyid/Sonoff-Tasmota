/*
  xdrv_93_electroConvector.ino - Elcectric heater thermostat based on PID algorithm plugin for Tasmota
  Copyright (C) 2018 Colin Law and Thomas Herrmann
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

/**
 * Code to
 *
 * Usage:
 * Place this file in the sonoff folder.
 * Clone the library https://github.com/colinl/process-control.git from Github
 * into a subfolder of lib.
 * If you want to use a time proportioned relay output with this then also get
 * xdrv_91_timeprop.ino
 * In user_config.h or user_config_override.h include code as follows:

 #define USE_PID         // include the pid feature (+4.3k)
   #define PID_SETPOINT                  19.5    // Setpoint value. This is the process value that the process is
                                                 // aiming for.
                                                 // May be adjusted via MQTT using cmnd pid_sp

   #define PID_PROPBAND                  5       // Proportional band in process units (eg degrees). This controls
                                                 // the gain of the loop and is the range of process value over which
                                                 // the power output will go from 0 to full power. The units are that
                                                 // of the process and setpoint, so for example in a heating
                                                 // application it might be set to 1.5 degrees.
                                                 // May be adjusted via MQTT using cmnd pid_pb

   #define PID_INTEGRAL_TIME             1800    // Integral time seconds. This is a setting for the integral time,
                                                 // in seconds. It represents the time constant of the integration
                                                 // effect. The larger the value the slower the integral effect will be.
                                                 // Obviously the slower the process is the larger this should be. For
                                                 // example for a domestic room heated by convection radiators a setting
                                                 // of one hour might be appropriate (in seconds). To disable the
                                                 // integral effect set this to a large number.
                                                 // May be adjusted via MQTT using cmnd pid_ti

   #define PID_DERIVATIVE_TIME           15      // Derivative time seconds. This is a setting for the derivative time,
                                                 // in seconds. It represents the time constant of the derivative effect.
                                                 // The larger the value the greater will be the derivative effect.
                                                 // Typically this will be set to somewhat less than 25% of the integral
                                                 // setting, once the integral has been adjusted to the optimum value. To
                                                 // disable the derivative effect set this to 0. When initially tuning a
                                                 // loop it is often sensible to start with derivative zero and wind it in
                                                 // once other parameters have been setup.
                                                 // May be adjusted via MQTT using cmnd pid_td

   #define PID_INITIAL_INT               0.5     // Initial integral value (0:1). This is an initial value which is used
                                                 // to preset the integrated error value when the flow is deployed in
                                                 // order to assist in homing in on the setpoint the first time. It should
                                                 // be set to an estimate of what the power requirement might be in order
                                                 // to maintain the process at the setpoint. For example for a domestic
                                                 // room heating application it might be set to 0.2 indicating that 20% of
                                                 // the available power might be required to maintain the setpoint. The
                                                 // value is of no consequence apart from device restart.

   #define PID_MAX_INTERVAL              300     // This is the maximum time in seconds that is expected between samples.
                                                 // It is provided to cope with unusual situations such as a faulty sensor
                                                 // that might prevent the node from being supplied with a process value.
                                                 // If no new process value is received for this time then the power is set
                                                 // to the value defined for PID_MANUAL_POWER.
                                                 // May be adjusted via MQTT using cmnd pid_max_interval

   #define PID_DERIV_SMOOTH_FACTOR       3       // In situations where the process sensor has limited resolution (such as
                                                 // the DS18B20), the use of deriviative can be problematic as when the
                                                 // process is changing only slowly the steps in the value cause spikes in
                                                 // the derivative. To reduce the effect of these this parameter can be
                                                 // set to apply a filter to the derivative term. I have found that with
                                                 // the DS18B20 that a value of 3 here can be beneficial, providing
                                                 // effectively a low pass filter on the derivative at 1/3 of the derivative
                                                 // time. This feature may also be useful if the process value is particularly
                                                 // noisy. The smaller the value the greater the filtering effect but the
                                                 // more it will reduce the effectiveness of the derivative. A value of zero
                                                 // disables this feature.
                                                 // May be adjusted via MQTT using cmnd pid_d_smooth

   #define PID_AUTO                      1       // Auto mode 1 or 0 (for manual). This can be used to enable or disable
                                                 // the control (1=enable, auto mode, 0=disabled, manual mode). When in
                                                 // manual mode the output is set the value definded for PID_MANUAL_POWER
                                                 // May be adjusted via MQTT using cmnd pid_auto

   #define PID_MANUAL_POWER              0       // Power output when in manual mode or fallback mode if too long elapses
                                                 // between process values
                                                 // May be adjusted via MQTT using cmnd pid_manual_power

   #define PID_UPDATE_SECS               0       // How often to run the pid algorithm (integer secs) or 0 to run the algorithm
                                                 // each time a new pv value is received, for most applictions specify 0.
                                                 // Otherwise set this to a time
                                                 // that is short compared to the response of the process.  For example,
                                                 // something like 15 seconds may well be appropriate for a domestic room
                                                 // heating application.
                                                 // May be adjusted via MQTT using cmnd pid_update_secs

   #define PID_USE_TIMPROP               1       // To use an internal relay for a time proportioned output to drive the
                                                 // process, set this to indicate which timeprop output to use. For a device
                                                 // with just one relay then this will be 1.
                                                 // It is then also necessary to define USE_TIMEPROP and set the output up as
                                                 // explained in xdrv_91_timeprop.ino
                                                 // To disable this feature leave this undefined (undefined, not defined to nothing).

   #define PID_USE_LOCAL_SENSOR                  // if defined then the local sensor will be used for pv. Leave undefined if
                                                 // this is not required.  The rate that the sensor is read is defined by TELE_PERIOD
                                                 // If not using the sensor then you can supply process values via MQTT using
                                                 // cmnd pid_pv

 * Help with using the PID algorithm and with loop tuning can be found at
 * http://blog.clanlaw.org.uk/2018/01/09/PID-tuning-with-node-red-contrib-pid.html
 * This is directed towards using the algorithm in the node-red node node-red-contrib-pid but the algorithm here is based on
 * the code there and the tuning techique described there should work just the same.

 *
**/

#ifdef USE_ELECTRO_CONVECTOR_HEATER

# include "PID.h"
# include "Timeprop.h"

#define D_CMND_PID "pid_"

#define D_CMND_PID_SETPV "pv"
#define D_CMND_PID_SETSETPOINT "sp"
#define D_CMND_PID_SETPROPBAND "pb"
#define D_CMND_PID_SETINTEGRAL_TIME "ti"
#define D_CMND_PID_SETDERIVATIVE_TIME "td"
#define D_CMND_PID_SETINITIAL_INT "initint"
#define D_CMND_PID_SETDERIV_SMOOTH_FACTOR "d_smooth"
#define D_CMND_PID_SETAUTO "auto"
#define D_CMND_PID_SETMANUAL_POWER "manual_power"
#define D_CMND_PID_SETMAX_INTERVAL "max_interval"
#define D_CMND_PID_SETUPDATE_SECS "update_secs"

enum PIDCommands { CMND_PID_SETPV, CMND_PID_SETSETPOINT, CMND_PID_SETPROPBAND, CMND_PID_SETINTEGRAL_TIME,
  CMND_PID_SETDERIVATIVE_TIME, CMND_PID_SETINITIAL_INT, CMND_PID_SETDERIV_SMOOTH_FACTOR, CMND_PID_SETAUTO,
  CMND_PID_SETMANUAL_POWER, CMND_PID_SETMAX_INTERVAL, CMND_PID_SETUPDATE_SECS };
const char kPIDCommands[] PROGMEM = D_CMND_PID_SETPV "|" D_CMND_PID_SETSETPOINT "|" D_CMND_PID_SETPROPBAND "|"
  D_CMND_PID_SETINTEGRAL_TIME "|" D_CMND_PID_SETDERIVATIVE_TIME "|" D_CMND_PID_SETINITIAL_INT "|" D_CMND_PID_SETDERIV_SMOOTH_FACTOR "|"
  D_CMND_PID_SETAUTO "|" D_CMND_PID_SETMANUAL_POWER "|" D_CMND_PID_SETMAX_INTERVAL "|" D_CMND_PID_SETUPDATE_SECS;

static PID pid;
static int update_secs = PID_UPDATE_SECS <= 0  ?  0  :  PID_UPDATE_SECS;   // how often (secs) the pid alogorithm is run
static int max_interval = PID_MAX_INTERVAL;
static unsigned long last_pv_update_secs = 0;
static boolean run_pid_now = false;     // tells PID_Every_Second to run the pid algorithm
static boolean reinit_timeprop = false;
static double pid_output;

static bool heater_enabled = true;
static bool heater_configured; 

/* ********************** TimeProp function ******************** */
static uint16_t enabledStages; /* bitfield */
static bool timeprop_enabled = true;
static int outPin[2];
static Timeprop timeprops[TIMEPROP_NUM_OUTPUTS];
static uint16_t currentRelayStates = 0;  // current actual relay states. Bit 0 first relay



static  double outpower[2] = { 400, 1250 };
static  double powerDistr[2] = { outpower[0] / (outpower[0] + outpower[1]), outpower[1] / (outpower[0] + outpower[1]) };  // 0.33, 0.66
static  double powerRate = outpower[0] / outpower[1];          



/******************* T I M E  P R O P ************************************/

void TimePropInit()
{
  snprintf_P(log_data, sizeof(log_data), "Timeprop Init");
  AddLog(LOG_LEVEL_INFO);
  int cycleTimes[TIMEPROP_NUM_OUTPUTS] = {TIMEPROP_CYCLETIMES};
  int deadTimes[TIMEPROP_NUM_OUTPUTS] = {TIMEPROP_DEADTIMES};
  int opInverts[TIMEPROP_NUM_OUTPUTS] = {TIMEPROP_OPINVERTS};
  int fallbacks[TIMEPROP_NUM_OUTPUTS] = {TIMEPROP_FALLBACK_POWERS};
  int maxIntervals[TIMEPROP_NUM_OUTPUTS] = {TIMEPROP_MAX_UPDATE_INTERVALS};

  for (int i=0; i<TIMEPROP_NUM_OUTPUTS; i++) {
    timeprops[i].initialise(cycleTimes[i], deadTimes[i], opInverts[i], fallbacks[i],
      maxIntervals[i], UtcTime());
  }
}

void Timeprop_Every_Second() {
  for (int i=0; i<TIMEPROP_NUM_OUTPUTS; i++) {
    int newState = timeprops[i].tick(UtcTime());
    if (newState != bitRead(currentRelayStates, i)){
      ExecuteCommandPower(
        i + 1, 
        newState ? POWER_ON : POWER_OFF, 
        SRC_HEATER);

/*      
      if (newState) {
        digitalWrite(outPin[i], HIGH);
        bitSet(currentRelayStates, i);
      }
      else {
        digitalWrite(outPin[i], LOW);
        bitClear(currentRelayStates, i);
      }

      XdrvMailbox.index = currentRelayStates;
      XdrvCall(FUNC_SET_POWER);
*/
    }
  }
}

void TimePropShowSensor(bool json)
{
  char tmpStr[33];

  if (json)
  {
    Response_P("");
    ResponseAppendTime();
  }

  for (int i = 0; i < TIMEPROP_NUM_OUTPUTS; i++)
  {
    if (json) /* --- MQTT resposne --- */
    {

      ResponseAppend_P(PSTR(",\"Output_%d\":{"), i);

        dtostrfd(timeprops[i].getEffectivePower(), 3, tmpStr);
        ResponseAppend_P(PSTR("\"EffectivePower\":\"%s\","), tmpStr);

        dtostrfd(timeprops[i].getRemainWindowTime(), 3, tmpStr);
        ResponseAppend_P(PSTR("\"RemainWindowTime\":\"%s\""), tmpStr);

      ResponseAppend_P(PSTR("}"));
    }
    else /* --- WEB sensor --- */
    {
      dtostrfd(timeprops[i].getRemainWindowTime(), 3, tmpStr);
      WSContentSend_PD(PSTR("{s}Propotional window %d:{m}%s{e}"), i, tmpStr);  // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>

      dtostrfd(timeprops[i].getEffectivePower(), 3, tmpStr);
      WSContentSend_PD(PSTR("{s}Effective Power %d:{m}%s{e}"), i, tmpStr);  // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
    }
  }

  if (json)
  {
    ResponseAppend_P(PSTR("}"));
  }
}



/* call this from elsewhere if required to set the power value for one of the timeprop instances */
/* index specifies which one, 0 up */
void Timeprop_Set_Power( int index, float power, boolean actNow )
{
  if (index >= 0  &&  index < TIMEPROP_NUM_OUTPUTS)
  {
    if (actNow) { timeprops[index].ReSetPower( power, UtcTime()); }
    else        { timeprops[index].setPower  ( power, UtcTime()); }
  }
}

// called by the system each time a relay state is changed
void Timeprop_Xdrv_Power() {
  // for a single relay the state is in the lsb of index, I have think that for
  // multiple outputs then succesive bits will hold the state but have not been
  // able to test that
  currentRelayStates = XdrvMailbox.index;
}


/**************************** P I D *************************************/


void PID_Init()
{
  /* no heater stage configure -> skip PID init and disable function */
  if (!heater_configured) {
    heater_enabled = false;
    return;
  }


  snprintf_P(log_data, sizeof(log_data), "PID Init");
  AddLog(LOG_LEVEL_INFO);
  pid.initialise( PID_SETPOINT, PID_PROPBAND, PID_INTEGRAL_TIME, PID_DERIVATIVE_TIME, PID_INITIAL_INT,
    PID_MAX_INTERVAL, PID_DERIV_SMOOTH_FACTOR, PID_AUTO, PID_MANUAL_POWER );

}

void PID_Every_Second() {
  static int sec_counter = 0;


  // run the pid algorithm if run_pid_now is true or if the right number of seconds has passed or if too long has
  // elapsed since last pv update. If too long has elapsed the the algorithm will deal with that.
  if (( update_secs != 0 ) && ( sec_counter % update_secs == 0) )
  {
    run_pid_now = true;
    sec_counter = 0;
  }
  sec_counter++;

  if (run_pid_now  ||  UtcTime() - last_pv_update_secs > max_interval ) {
    run_pid();
    run_pid_now = false;
  }

}

void PID_Show_Sensor() {
  // Called each time new sensor data available, data in mqtt data in same format
  // as published in tele/SENSOR
  // Update period is specified in TELE_PERIOD
  // e.g. "{"Time":"2018-03-13T16:48:05","DS18B20":{"Temperature":22.0},"TempUnit":"C"}"
  snprintf_P(log_data, sizeof(log_data), "PID_Show_Sensor: mqtt_data: %s", mqtt_data);
  AddLog(LOG_LEVEL_INFO);

  StaticJsonBuffer<1024> jsonBuffer;
  String jsonStr = mqtt_data;  // Move from stack to heap to fix watchdogs (20180626)
  // force mqtt_data to read only to stop parse from overwriting it
  JsonObject &data_json = jsonBuffer.parseObject(jsonStr);
  if (data_json.success()) {
    const char* value = NULL;

    value = data_json[D_SENSOR_DHT11]["Temperature"];
    if (value == NULL) { value = data_json[D_SENSOR_AM2301]["Temperature"]; }
    else if (value == NULL) { value = data_json[D_SENSOR_SI7021]["Temperature"]; }

    // check that something was found and it contains a number
    if (value != NULL && strlen(value) > 0 && (isdigit(value[0]) || (value[0] == '-' && isdigit(value[1])) ) ) {
      snprintf_P(log_data, sizeof(log_data), "PID_Show_Sensor: Temperature: %s", value);
      AddLog(LOG_LEVEL_INFO);
      // pass the value to the pid alogorithm to use as current pv
      last_pv_update_secs = UtcTime();
      pid.setPv(atof(value), last_pv_update_secs);
      // also trigger running the pid algorithm if we have been told to run it each pv sample
      if (update_secs == 0) {
        // this runs it at the next second
        run_pid_now = true;
      }
    } else {
      snprintf_P(log_data, sizeof(log_data), "PID_Show_Sensor - no temperature found");
      AddLog(LOG_LEVEL_INFO);
    }
  } else  {
    // parse failed
    snprintf_P(log_data, sizeof(log_data), "PID_Show_Sensor - json parse failed");
    AddLog(LOG_LEVEL_INFO);
  }

#ifdef IOT_GURU_BASE_URL
  if (!(
        (Settings.iotGuruNodeKey[0] == 0x00)
        ||
        (Settings.iotGuruNodeKey[0] == '-' && Settings.iotGuruNodeKey[1] == 0x00)
        )
      )
  {
    HTTPClient httpClient;
    String nodeKey = String(Settings.iotGuruNodeKey);
    String url;
    int code;
    char tmpStr[10];

    dtostrfd(pid_output * (outpower[0] + outpower[1]), 1, tmpStr);
    url = String(IOT_GURU_BASE_URL) + "measurement/create/" + nodeKey +
          "/heater_power/" + String(tmpStr);

    httpClient.useHTTP10(true);
    httpClient.setTimeout(1000);

    yield();
    httpClient.begin(url);
    code = httpClient.GET();
    httpClient.end();
    yield();

    AddLog_P2(LOG_LEVEL_DEBUG, PSTR("url:%s"), url.c_str());
    AddLog_P2(LOG_LEVEL_DEBUG, PSTR("PID power send. exitcode=%d"), code);
  }
#endif

}


/* struct XDRVMAILBOX { */
/*   uint16_t      valid; */
/*   uint16_t      index; */
/*   uint16_t      data_len; */
/*   int16_t       payload; */
/*   char         *topic; */
/*   char         *data; */
/* } XdrvMailbox; */

boolean PID_Command()
{
  char command [CMDSZ];
  boolean serviced = true;
  uint8_t ua_prefix_len = strlen(D_CMND_PID); // to detect prefix of command

  AddLog_P2(LOG_LEVEL_DEBUG, 
    PSTR("Command called: index: %d data_len: %d payload: %d topic: %s data: %s"),
    XdrvMailbox.index,
    XdrvMailbox.data_len,
    XdrvMailbox.payload,
    (XdrvMailbox.payload >= 0 ? XdrvMailbox.topic : ""),
    (XdrvMailbox.data_len >= 0 ? XdrvMailbox.data : ""));

  if (0 == strncasecmp_P(XdrvMailbox.topic, PSTR(D_CMND_PID), ua_prefix_len)) {
    // command starts with pid_
    int command_code = GetCommandCode(command, sizeof(command), XdrvMailbox.topic + ua_prefix_len, kPIDCommands);
    serviced = true;
    switch (command_code) {
      case CMND_PID_SETPV:
        last_pv_update_secs = UtcTime();
        pid.setPv(atof(XdrvMailbox.data), last_pv_update_secs);
        // also trigger running the pid algorithm if we have been told to run it each pv sample
        if (update_secs == 0) {
          // this runs it at the next second
          run_pid_now = true;
        }
        AddLog_P2(LOG_LEVEL_INFO, "PID command setpv");
        break;

      case CMND_PID_SETSETPOINT:
        pid.setSp(atof(XdrvMailbox.data));
        // also trigger new pid calculation
        run_pid_now = true;
        reinit_timeprop = true;
        break;

      case CMND_PID_SETPROPBAND:
        pid.setPb(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETINTEGRAL_TIME:
        pid.setTi(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETDERIVATIVE_TIME:
        pid.setTd(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETINITIAL_INT:
        pid.setInitialInt(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETDERIV_SMOOTH_FACTOR:
        pid.setDSmooth(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETAUTO:
        pid.setAuto(atoi(XdrvMailbox.data));
        break;

      case CMND_PID_SETMANUAL_POWER:
        pid.setManualPower(atof(XdrvMailbox.data));
        break;

      case CMND_PID_SETMAX_INTERVAL:
      max_interval = atoi(XdrvMailbox.data);
      pid.setMaxInterval(max_interval);
      break;

      case CMND_PID_SETUPDATE_SECS:
        update_secs = atoi(XdrvMailbox.data) ;
        if (update_secs < 0) update_secs = 0;
        break;

      default:
        serviced = false;
  }

    if (serviced) {
      // set mqtt RESULT
      snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("{\"%s\":\"%s\"}"), XdrvMailbox.topic, XdrvMailbox.data);
    }

  } else {
    serviced = false;
  }
  return serviced;
}

static void run_pid()
{
  double power = pid.tick(UtcTime());
  char buf[10];
  pid_output = power;

  /* Report only in case of normal operation, if sensor value missing, prevent 1s repetition */
  if (run_pid_now)
  {
    dtostrfd(power, 3, buf);

    Response_P("");
    ResponseAppendTime();
    ResponseAppend_P(PSTR(",\"power\":%s"), buf);
    ResponseJsonEnd();
    MqttPublishPrefixTopic_P(TELE, "PID", false);
  }

#if defined PID_USE_TIMPROP
  // send power to appropriate timeprop output
  // calculate the control band for 2 stage electric heaters
  double duty[2] = { 0.0, 0.0 };


  if (power < 0.01)
  {
    duty[0] = 0.0;
    duty[1] = 0.0;
  }
  else if (power <= powerDistr[0])   /* 0.0 --- 0.33  PWM500*/
  {
    duty[0] = mapfloat(power, 0.0, powerDistr[0], 0.0, 1.0);
    duty[1] = 0.0;
  }
  else if (power <= powerDistr[1])  /* 0.331 --- 0.66   PWM1000*/
  {
    duty[0] = 0.0;
    duty[1] = mapfloat(power, powerDistr[0], powerDistr[1], powerRate, 1.0);
  }
  else if (power <= 1.0)  /* 0.825 --- 1.0   PWM1000 + ON500*/
  {
    duty[0] = 1.0;
    duty[1] = mapfloat(power, powerDistr[1], 1.0, 1 - powerRate, 1.0);
  }
  else
  {
    duty[0] = 1.0;
    duty[1] = 1.0;
  }

  Timeprop_Set_Power( 0, duty[0], reinit_timeprop );
  Timeprop_Set_Power( 1, duty[1], reinit_timeprop );
  reinit_timeprop = false;
#endif // PID_USE_TIMPROP
}


bool eConvectorPinState()
{
  bool ret = false;
  int relayId;

  if ( (XdrvMailbox.index >= GPIO_HEATER_STAGE1) && (XdrvMailbox.index <= GPIO_HEATER_STAGE2) )
  {
    relayId = XdrvMailbox.index - GPIO_HEATER_STAGE1;
    outPin[relayId] = XdrvMailbox.payload;

    XdrvMailbox.index = GPIO_REL1 + relayId;  /* set the output as default relay */

    heater_configured = true;
    ret = true;

    AddLog_P2(LOG_LEVEL_INFO,
              PSTR("HEAT: Stage%d on pin: %d activated"),
              relayId, outPin[relayId]);
  }
  return ret;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

#define XDRV_93  93

boolean Xdrv93(byte function)
{
  boolean result = false;
  char tmpStr[33];

  /* the function is not configured skip it */
  if (!heater_enabled)
  {
    return false;
  }

  switch (function) {
    case FUNC_INIT:
      PID_Init();
      TimePropInit();
      break;
    case FUNC_EVERY_SECOND:
      PID_Every_Second();
      Timeprop_Every_Second();
      break;
    case FUNC_SET_POWER:
      Timeprop_Xdrv_Power();
      TimePropShowSensor(true);
      break;
    case FUNC_SHOW_SENSOR:
      // only use this if the pid loop is to use the local sensor for pv
      #if defined PID_USE_LOCAL_SENSOR
        PID_Show_Sensor();
      #endif // PID_USE_LOCAL_SENSOR
      TimePropShowSensor(true);
      break;
  #ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      TimePropShowSensor(false);

      dtostrfd(pid_output, 3, tmpStr);
      WSContentSend_PD(PSTR("{s}PID factor:{m}%s{e}"), tmpStr);  // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
      dtostrfd(pid_output * (outpower[0] + outpower[1]), 1, tmpStr);
      WSContentSend_PD(PSTR("{s}Needed power:{m}%s " D_UNIT_WATT "{e}"), tmpStr);  // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
      dtostrfd(pid.getSp(), 1, tmpStr);
      WSContentSend_PD(PSTR("{s}Thermostat setpoint:{m}%s&deg;{e}"), tmpStr);  // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
    break;
  #endif
    case FUNC_COMMAND:
      result = PID_Command();
      break;

    case FUNC_PIN_STATE:
      result = eConvectorPinState();
      break;
  }
  return result;
}

#endif // USE_ELECTRO_CONVECTOR_HEATER