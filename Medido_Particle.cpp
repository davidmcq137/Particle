/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/home/davidmcq/particle/Medido_Particle/src/Medido_Particle.ino"
/*
 * Project Medido_Particle
 * Description: Control code for Medido Pump .. Particle Argon
 * Author: D. McQueeney
 * Date: 14-May-2020
 */

#include <Particle.h>

void powerDownTimeout();
void loop();
void setup();
void gpioCBFill();
void gpioCBEmpty();
void gpioCBStop();
void setRunSpeed(int pw);
void setPumpSpeed(float ps);
void setPumpFwd();
void setPumpRev();
void sendSPI(String str, float val);
float adcVolts(float vRaw);
void timerCB();
void onLinefeed(String msg);
void execKwd(String k, String v);
void execCmd(String k, String v);
#line 10 "/home/davidmcq/particle/Medido_Particle/src/Medido_Particle.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

float PIDpGain = 0.005;
float PIDiGain = 2;
float PIDpTerm = 0;
float PIDiTerm = 0;
float MINpress = 0;
float MAXpress = 10;
float pressLimit = (MAXpress + MINpress) / 2;
float pulsePerOzFill = 100.0;
float pulsePerOzEmpty = 100.0;
int dispRstPin = D6;
int flowMeterPinFill = A2;
int flowMeterPinEmpty = A3;
int flowMeterPinStop = A4;
int powerDownPin = D5;
int pwmPumpPin = D7;
int flowDirPin = D8;
volatile int pulseCountFill = 0;
volatile int pulseCountEmpty = 0;
volatile int pulseCountStop = 0;
int pulseCountBad = 0;
int pulseStop = 10; // set # pulses to stop pump on piss tank sensor @77 pulses/oz this is 3.5 ml
int flowRate = 0;
int lastPulseCountFill = 0;
int lastPulseCountEmpty = 0;
int lastPulseCountBad = 0;
int lastFlowTime = 0;
float pressZero;
float pressScale = 3.75;
float currentZero = 0.0;
int minPWM = 50;
int maxPWM = 1023;
int opPWM = maxPWM;
int pumpPWM = 0;
int runPWM = 0;
int pressPSI = 0;
bool pumpFwd = true;
int saveSetSpeed = 0;
unsigned long pumpStartTime = 0;
unsigned long pumpStopTime = 0;
float runningTime = 0.0;
int flowCount = 0;
int pumpTimer = 0;
int watchTimer = 0;
int powerOffMins = 30;
unsigned long bootTime = 0;
int adcZero = 0;
float adcAvg = 0.0;
float adcScale = 1240.9; // 4095 / 3.3
float adcDiv = 1.666;    // resistive divider in front of Argon board adc from pressure sensor
int dw = 128;
int dh = 64;
int seq = 0;
unsigned long lastLoop = 0;
unsigned long loopMinTime = 50;
bool medidoEnabled = true;

const size_t UART_TX_BUF_SIZE = 20;
void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context);
String textacc;

// These UUIDs were defined by Nordic Semiconductor and are now the defacto standard for
// UART-like services over BLE. Many apps support the UUIDs now, like the Adafruit Bluefruit app.

const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid rxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY, txUuid, serviceUuid);
//BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE_WO_RSP, rxUuid, serviceUuid, onDataReceived, NULL);
BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE, rxUuid, serviceUuid, onDataReceived, NULL);

float loopTimAvg = 0;
unsigned long loopTimLast = 0;
unsigned long loopDelta = 0;
int kk;

void powerDownTimeout()
{
  sendSPI("PowerDown", 0.0);
  delay(1s); // wait for message to get to ios
  //Serial.println("timeout!");
  digitalWrite(powerDownPin, HIGH);
}

Timer powerTimer(1000 * 60 * powerOffMins, powerDownTimeout);

//Timer mainLoop(200, timerCB);

// loop() runs over and over again, as quickly as it can execute.
void loop()
{

  Particle.process();

  if (medidoEnabled)
  {

    if (loopTimLast != 0)
    {
      loopDelta = micros() - loopTimLast;
      //Serial.println(loopDelta);
      loopTimAvg = loopTimAvg + (float)(loopDelta - loopTimAvg) / 10.0;
      //Serial.println(loopTimAvg);
    }
    loopTimLast = micros();

    //Serial.print(millis());
    //Serial.print(lastLoop);
    //Serial.println(millis() - lastLoop);

    if ((int)((unsigned long)millis() - lastLoop) > (int)loopMinTime)
    {

      //Serial.print("Loop avg time: ");
      //Serial.println((float)loopTimAvg);
      //Serial.print(millis());
      //Serial.print(" ");
      //Serial.println((unsigned long)millis() - lastLoop);
      timerCB();

      lastLoop = millis();
      //Serial.print("kk: ");
      //Serial.println(kk);
      kk = 0;
    }
    else
    {
      kk = kk + 1;
    }

    if (BLE.connected())
    {
      uint8_t txBuf[UART_TX_BUF_SIZE];
      size_t txLen = 0;

      while (Serial.available() && txLen < UART_TX_BUF_SIZE)
      {
        txBuf[txLen++] = Serial.read();
        Serial.write(txBuf[txLen - 1]);
      }
      if (txLen > 0)
      {
        txCharacteristic.setValue(txBuf, txLen);
      }
    }
  }
}

// setup() runs once, when the device is first turned on.
void setup()
{
  bootTime = millis();
  Serial.begin();
  BLE.on();
  Serial.println("starting setup");

  powerTimer.start();

  BLE.addCharacteristic(txCharacteristic);
  BLE.addCharacteristic(rxCharacteristic);

  BleAdvertisingData data;
  data.appendServiceUUID(serviceUuid);
  BLE.advertise(&data);

  pinMode(pwmPumpPin, OUTPUT);
  analogWriteResolution(pwmPumpPin, 10);
  analogWrite(pwmPumpPin, 0, 1000);

  pinMode(flowDirPin, OUTPUT);

  // Preset pump speed to 0, set FWD direction

  setPumpSpeed(0);
  setPumpFwd();

  // Get a zero cal point on the current sensor

  for (int i = 1; i <= 10; i++)
  {
    currentZero = currentZero + (float)analogRead(A5);
  }

  currentZero = currentZero / 10.0;

  // Get a zero cal point on the pressure transducer

  int arsum = 0;
  int ar = 0;
  for (int i = 1; i <= 50; i++)
  {
    ar = analogRead(A0);
    Serial.println(ar);
    Serial.println((float)ar * 3.3 / 4096.);
    arsum = arsum + ar;
  }
  Serial.printlnf("arsum: %d", arsum);
  Serial.printlnf("(float)arsum / 50.0: %f", (float)arsum / 50.0);
  pressZero = adcVolts((float)arsum / 50.0);
  Serial.printlnf("pressZero: %f", pressZero);

  // Set up interrupts to catch pulses from the flowmeters

  pinMode(flowMeterPinFill, INPUT_PULLUP);
  attachInterrupt(flowMeterPinFill, gpioCBFill, FALLING);

  pinMode(flowMeterPinEmpty, INPUT_PULLUP);
  attachInterrupt(flowMeterPinEmpty, gpioCBEmpty, FALLING);

  pinMode(flowMeterPinStop, INPUT_PULLUP);
  attachInterrupt(flowMeterPinStop, gpioCBStop, FALLING);

  // Setup power down pin, taking it high turns off the pump

  pinMode(powerDownPin, OUTPUT);
  digitalWrite(powerDownPin, LOW);

  // Setup a pin to reset the OLED display

  pinMode(dispRstPin, OUTPUT);

  sendSPI("Init", 0);
  sendSPI("rPWM", 0);

  // mainLoop.start();

  Serial.println("init done");
}

void gpioCBFill()
{
  if (pumpFwd)
  {
    pulseCountFill += 1;
  }
  else
  {
    pulseCountBad += 1;
  }
}

void gpioCBEmpty()
{
  if (!pumpFwd)
  {
    pulseCountEmpty += 1;
  }
  else
  {
    pulseCountBad += 1;
  }
}

void gpioCBStop()
{
  // code to stop goes here
  pulseCountStop += 1;
}

void setRunSpeed(int pw)
{ // careful .. this is in pwm units 0 .. 1023 ..  not %
  pulseCountBad = 0;
  lastPulseCountBad = 0;

  sendSPI("rPWM", pw);

  analogWrite(pwmPumpPin, pw);
  Serial.print("just sent pw: ");
  Serial.println(pw);

  runPWM = pw;
}

int oldspd = 0;

void setPumpSpeed(float ps)
{                                      // this is ps units // 0 to 100%
  pumpPWM = (int)(ps * opPWM / 100.0); //math.floor(ps*opPWM/100)
  if (pumpPWM < minPWM)
  {
    pumpPWM = 0;
  }
  if (pumpPWM > maxPWM)
  {
    pumpPWM = maxPWM;
  }
  sendSPI("pPWM", pumpPWM);
  if (pumpPWM != oldspd)
  {
    //print("pump speed set to", pumpPWM)
  }
  oldspd = pumpPWM;
}

void setPumpFwd()
{
  digitalWrite(flowDirPin, LOW);
  pumpFwd = true;
  PIDiTerm = saveSetSpeed * minPWM / maxPWM; // setting to 1.0x saveSetSpeed caused overshoot
  pulseCountStop = 0;                        // reset piss tank flow sensor
  if (pumpPWM > 0)
  { // just turned on .. note startime
    pumpStartTime = millis();
  }
  //start the pump at min speed .. let the PID ramp it up
  //setRunSpeed(pumpPWM)
  if (pumpPWM < minPWM)
  {
    setRunSpeed(pumpPWM);
  }
  else
  {
    setRunSpeed(minPWM);
  }
}

void setPumpRev()
{
  digitalWrite(flowDirPin, HIGH);
  pumpFwd = false;
  PIDiTerm = 0;
  pulseCountStop = 0; // reset piss tank flow sensor
  if (pumpPWM > 0)
  { // just turned on .. note startime
    pumpStartTime = millis();
  }
  setRunSpeed(pumpPWM);
}

void sendSPI(String str, float val)
{
  {
    size_t nch;
    char buf[20];
    uint8_t txbuf[20];

    nch = sprintf(buf, "(" + str + ":%4.2f)", val);
    for (size_t ii = 0; ii < nch; ii++)
    {
      txbuf[ii] = (int)buf[ii];
    }
    txCharacteristic.setValue(txbuf, nch);
  }
}

float adcVolts(float vRaw)
{
  float v;
  v = adcDiv * (vRaw - adcZero) / adcScale;
  //Serial.print("vRaw: ");
  //Serial.print(vRaw);
  //Serial.print(" volts: ");
  //Serial.println(v);

  return v;
}

void timerCB()
{

  float pSpd;
  float errsig;
  float deltaFFill;
  float deltaFEmpty;
  float deltaF;
  unsigned long now;
  float deltaT;
  unsigned long dtms;

  // send the app a message if we get #pulses > pulseStop  at piss tank sensor

  if (pulseCountStop > pulseStop)
  {
    sendSPI("pSTP", pulseCountStop);
    pulseCountStop = 0;
  }

  adcAvg = adcAvg - (adcAvg - (float)analogRead(A0)) / 4.0; // running avg of raw adc readings
  pressPSI = (adcVolts(adcAvg) - pressZero) * (pressScale);
  sendSPI("pPSI", pressPSI);

  //if pressPSI < 0 then pressPSI = 0 end

  flowCount = (pulseCountFill / pulsePerOzFill) - (pulseCountEmpty / pulsePerOzEmpty);

  sendSPI("fCNT", flowCount);

  if (pulseCountBad != lastPulseCountBad)
  {
    sendSPI("cBAD", pulseCountBad);
    lastPulseCountBad = pulseCountBad;
  }

  now = millis();

  if (pumpStartTime == 0)
  {
    pumpStartTime = now;
  }

  dtms = now - lastFlowTime;
  //  if needed port this line: if dtms < 0 then dtus = dtus + 2147483647 end // in case of rollover
  if (dtms > 1000)
  {
    deltaT = (float)dtms / (1000. * 60.); // mins
    deltaFFill = (pulseCountFill - lastPulseCountFill) / pulsePerOzFill;
    deltaFEmpty = (pulseCountEmpty - lastPulseCountEmpty) / pulsePerOzEmpty;
    deltaF = deltaFFill - deltaFEmpty;
    flowRate = flowRate - (flowRate - (deltaF / deltaT)) / 1.2;
    sendSPI("fRAT", flowRate);
    sendSPI("fDEL", deltaF);
    sendSPI("fDET", deltaT);
    lastPulseCountFill = pulseCountFill;
    lastPulseCountEmpty = pulseCountEmpty;
    lastFlowTime = now;
  }

  if (runPWM > 0)
  {
    if (now > pumpStartTime)
    {                                                      //sometimes can have few 100 ms skew that makes now - pumpStartTime negative .. just set to zero
      runningTime = (float)(now - pumpStartTime) / 1000.0; // secs
    }
    else
    {
      runningTime = 0.0;
    }
    if (runningTime > 100000.0 || runningTime < 0.0)
    {
      Serial.printlnf("runPWM > 0: runPWM=%d, now=%lu, pumpStartTime=%lu, runningTime=%f", runPWM, now, pumpStartTime, runningTime);
    }
  }
  else
  {
    if (pumpStopTime > pumpStartTime)
    {
      runningTime = (float)(pumpStopTime - pumpStartTime) / 1000.0;
    }
    else
    {
      runningTime = 0.0;
    }
    if (runningTime > 100000.0 || runningTime < 0.0)
    {
      Serial.printlnf("runPWM <= 0: runPWM=%d, pumpStopTime=%lu, pumpStartTime=%lu, runningTime=%f", runPWM, pumpStopTime, pumpStartTime, runningTime);
    }
  }

  // dont send time if pump not on .. confuses graph on iPhone

  if (runPWM != 0)
  {
    sendSPI("rTIM", runningTime);
    //Serial.print("Running Time: ");
    //Serial.println(runningTime);
  }

  if ((pumpFwd && (runPWM > 0)) && (PIDiGain != 0 or PIDpGain != 0))
  {
    errsig = pressLimit - pressPSI;
    PIDpTerm = errsig * PIDpGain;
    PIDiTerm = PIDiTerm + errsig * PIDiGain;
    if (PIDiTerm < 0.0)
    {
      PIDiTerm = 0.0;
    }
    if (PIDiTerm > 100.0)
    {
      PIDiTerm = 100.0;
    }
    pSpd = PIDpTerm + PIDiTerm;
    if (pSpd < 0.0)
    {
      pSpd = 0.0;
    }
    if (pSpd > saveSetSpeed)
    {
      pSpd = saveSetSpeed;
    }
    setPumpSpeed(pSpd);   // side effect: sets pumpPWM within bounds
    setRunSpeed(pumpPWM); // side effect: sets runPWM
  }

  seq = seq + 1;
  if ((seq % 5 == 0) && (millis() - bootTime > 5000))
  {
    //lineLCD(2, "Flw", flowRate,  "%2.1f", " oz/s")
    //lineLCD(3, "Vol", flowCount, "%4.1f", " oz")
    if (runPWM == 0)
    {
      //lineLCD(4, timeFmt(0))
    }
    else
    {
      //lineLCD(4, timeFmt(runningTime))
    }
    //showLCD()

    if (seq % 10 == 0)
    {
      sendSPI("Batt", (float)analogRead(A1) * 0.5644 * 3.3 / 4096.0); // kludge: 8266 version has 7.504:1 divider, Argon version is 4.235 .. correct for that
                                                                      //should just send volts and change 8266 code and ios app..
      //sendSPI("Batt", 12.3/7.5);
    }
    if (seq % 10 == 6)
    {
      sendSPI("Curr", 100 * ((3.3 / 4096.0) * ((float)analogRead(A5) - currentZero)));
    }
  }
}

void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context)
{
  //Log.trace("Received data from: %02X:%02X:%02X:%02X:%02X:%02X:", peer.address()[0], peer.address()[1], peer.address()[2], peer.address()[3], peer.address()[4], peer.address()[5]);
  //Serial.print(len);

  //Serial.println("in onDataReceived");

  for (size_t ii = 0; ii < len; ii++)
  {
    //Serial.print(data[ii]);
    //Serial.print(char(data[ii]));
    textacc.concat(char(data[ii]));
    //Serial.write(data[ii]);
    //Serial.print("_");
    //Serial.print(ii);
    //Serial.print(">>" + textacc);
    //Serial.print("\r\n");
    if (data[ii] == '\n')
    {
      onLinefeed(textacc.c_str());
    }
  }
}

void onLinefeed(String msg)
{
  int lparen;
  int rparen;
  int colon;
  String key;
  String val;
  bool isKwd = false;

  //Serial.println("in onLinefeed");
  //Serial.println(msg);
  lparen = msg.indexOf("(");
  if (lparen < 0)
  {
    isKwd = true;
    rparen = msg.length() - 1;
  }
  else
  {
    rparen = msg.indexOf(")");
  }
  colon = msg.lastIndexOf(":");
  //Serial.print(lparen);
  //Serial.print(rparen);
  //Serial.print(colon);
  //Serial.println("%");

  //if ( (lparen < 0 || rparen < 0) && (!isKwd) ) {
  //  Serial.println("lparen and rparen < 0 returning");
  //  return;
  //}

  if (colon < 0)
  {
    key = msg.substring(lparen + 1, rparen);
    val = "";
  }
  else
  {
    key = msg.substring(lparen + 1, colon);
    val = msg.substring(colon + 1, rparen);
  }

  //Serial.print(key);
  //Serial.print("+++");
  //Serial.print(val);
  //Serial.println("###");

  textacc = "";

  if (isKwd)
  {
    execKwd(key, val);
  }
  else
  {
    execCmd(key, val);
  }
}

String wifissid = "";
String wifipwd = "";
String wifihost = "";
String wifidir = "";
String wifiimage = "";

void execKwd(String k, String v)
{
  String tmp = "";

  //Serial.printlnf("execKwd: k,v = %s,%s", k.c_str(), v.c_str());
  if (k == "ssid")
  {
    wifissid.concat(v);
  }
  else if (k == "pwd")
  {
    wifipwd.concat(v);
  }
  else if (k == "host")
  {
    wifihost.concat(v);
  }
  else if (k == "dir")
  {
    wifidir.concat(v);
  }
  else if (k == "image")
  {
    wifiimage.concat(v);
  }
  else if (k == "stop")
  {
    Serial.println("Stop cmd received");
  }
  else if (k == "update")
  {
    Serial.println("Update command received");
    medidoEnabled = false;
    WiFi.on();
    WiFi.connect();
    if (WiFi.ready())
    {
      Serial.println("WiFi.ready() returns true");
    }
    else
    {
      Serial.println("WiFi.ready() returns true");
    }
    Serial.print("ssid: ");
    Serial.println(wifissid.c_str());
    Serial.print("pwd: ");
    Serial.println(wifipwd);
    Serial.print("host: ");
    Serial.println(wifihost);
    Serial.print("dir: ");
    Serial.println(wifidir);
    Serial.print("image: ");
    Serial.println(wifiimage);
  }
}

void execCmd(String k, String v)
{

  // each time we get any command, restart the poweroff timer

  powerTimer.start(); // this will reset it if already running

  Serial.printlnf("execCmd: k,v = %s,%s", k.c_str(), v.c_str());

  if (k == "Spd")
  { // what change was it?
    saveSetSpeed = atof(v.c_str());
    //Serial.print("+Spd: ");
    //Serial.println(saveSetSpeed);
    setPumpSpeed(saveSetSpeed);
    if (saveSetSpeed == 0)
    {
      //lineLCD(1, "Pump Off")
    }
    //lineLCD(2, "Set Spd", saveSetSpeed, "%d", "%")
    //showLCD()
  }
  else if (k == "Fill")
  {
    setPumpFwd();
    //lineLCD()
    //lineLCD(1, "Fill")
    //lineLCD(2, "Set Spd", saveSetSpeed, "%d", "%")
    //showLCD()
  }
  else if (k == "Off")
  {
    //print("pump stop")
    setPumpSpeed(0.0);
    setRunSpeed(0);
    pumpStopTime = millis();
    Serial.printlnf("OFF cmd: runPWM, pumpPWM, pumpFwd: %d, %d", runPWM, pumpPWM);

    //lineLCD(1, "Pump Off")
  }
  else if (k == "Empty")
  {
    setPumpRev();
    //lineLCD()
    //lineLCD(1, "Empty")
  }
  else if (k == "Clear")
  {
    pulseCountFill = 0;
    pulseCountEmpty = 0;
    pulseCountBad = 0;

    lastPulseCountFill = 0;
    lastPulseCountEmpty = 0;
    lastPulseCountBad = 0;

    flowCount = 0;
    runningTime = 0;
    pumpStartTime = 0;
    pumpStopTime = 0;

    sendSPI("rTIM", runningTime);
    //lineLCD(1, "Clear")
  }
  else if (k == "CalF")
  {
    //print("CalFactFill passed in:", tonumber(v))
    pulsePerOzFill = atof(v.c_str()) / 10.0;
  }
  else if (k == "CalE")
  {
    //print("CalFactEmpty passed in:", tonumber(v))
    pulsePerOzEmpty = atof(v.c_str()) / 10.0;
  }
  else if (k == "Prs")
  {
    pressLimit = atof(v.c_str()) / 10.0;
    if (pressLimit > MAXpress)
    {
      pressLimit = MAXpress;
    }
    if (pressLimit < MINpress)
    {
      pressLimit = MINpress;
    }
  }
  else if (k == "PwrOff")
  {
    digitalWrite(powerDownPin, HIGH);
  }
  else if (k == "pMAX")
  {
    opPWM = atof(v.c_str());
    if (opPWM > maxPWM)
    {
      opPWM = maxPWM;
    }
    if (opPWM < minPWM)
    {
      opPWM = minPWM;
    }
  }
  else
  {
    Serial.printlnf("execCmd cmd error: k,v = %s,%s", k.c_str(), v.c_str());
  }
}