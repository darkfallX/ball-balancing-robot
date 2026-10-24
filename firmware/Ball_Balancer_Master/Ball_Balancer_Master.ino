#include <AccelStepper.h>
#include "InverseKinematics.h"
#include <MultiStepper.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

Machine machine(2, 3.125, 1.75, 3.669291339);
TouchScreen ts = TouchScreen(A2, A3, A0, A1, 0);

AccelStepper stepperA(1, 22, 23);
AccelStepper stepperB(1, 19, 20);
AccelStepper stepperC(1, 40, 41);
MultiStepper steppers;

const int EN1 = 21, EN2 = 18, EN3 = 39;

long   pos[3];
double angOrig   = 206.662752199;
double angToStep = 3200 / 360.0;
double speed[3] = {0,0,0}, speedPrev[3], ks = 20;
const double MAX_TRACK_SPEED = 1000.0;
const double SPEED_SLEW       = 200.0;

double Xoffset = 526.25, Yoffset = 517.38;
double filteredX = 526.25, filteredY = 517.38;
float  filterAlpha = 0.4;
double alignAngle = 0.2;
double tiltBiasX = 0.040, tiltBiasY = -0.035;
bool   detected = 0;
const unsigned long BALL_LOSS_TIMEOUT_MS = 250;
unsigned long lastBallSeen = 0;
bool trajectorySuspended = true;

double error[2] = {0,0}, errorPrev[2] = {0,0};
double integr[2] = {0,0}, deriv[2] = {0,0}, out[2] = {0,0};
const double OUT_CLAMP = 0.15;
const double INTEGRAL_LIMIT = 200000.0;
long timeI;

double kp = 3.5E-4, ki = 2E-7, kd = 4.0E-3;

double kiTrim = 2.0E-7;

double K1 = 3.523E-4, K2 = 7.150E-3;

double fzEmax = 300.0;
double fzDmax = 15.0;
double fzUmax = 0.15;

double mpcQ1 = 1.0;
double mpcQ2 = 0.02;
double mpcR  = 100.0;
int    mpcN  = 8;
double mpcDt = 0.020;
double plantB = -7.007143;

bool manualTilt = false;
double manualNx = 0, manualNy = 0;
unsigned long manualUntil = 0;

enum Shape { SH_OFF=0, SH_POINT, SH_CIRCLE, SH_SQUARE, SH_TRIANGLE,
             SH_FIGURE8, SH_HEART, SH_LINE };
int    shape       = SH_OFF;
double shapeSize   = 80.0;     
double shapePeriod = 8.0;      
double shapeRamp   = 1.0;      
unsigned long shapeT0 = 0;

double trajectoryStart[2] = {0,0};
double lastSetpoint[2]     = {0,0};
double referenceVelocity[2] = {0,0};  
double referenceAccel[2]    = {0,0};  
double pointTarget[2] = {0,0};        

double pathPhase = 0.0;                 
double pathRateScale = 0.0;             
unsigned long pathLastUpdate = 0;

bool cornerHold = false;
unsigned long cornerStableSince = 0;
const unsigned long CORNER_DWELL_MS = 120;

const char* shapeName(int s){
  switch(s){
    case SH_OFF:     return "OFF";
    case SH_POINT:   return "POINT";
    case SH_CIRCLE:  return "CIRCLE";
    case SH_SQUARE:  return "SQUARE";
    case SH_TRIANGLE:return "TRIANGLE";
    case SH_FIGURE8: return "FIGURE8";
    case SH_HEART:   return "HEART";
    case SH_LINE:    return "LINE";
  }
  return "?";
}

const double X_COUNT_TO_M = 0.168 / 826.0; 
const double Y_COUNT_TO_M = 0.168 / 826.0;
const double PLANT_K    = 7.007143;        
double ffGain    = 0.0;                    
double ffClamp   = 0.06;
double ffAlpha   = 0.25;                   

double ffTilt[2]  = {0,0};

enum Ctrl { CTRL_PID=0, CTRL_LQR=1, CTRL_FUZZY=2, CTRL_MPC=3 };
int  activeCtrl = CTRL_PID;
bool ctrlReady[4] = { true, true, true, true };

unsigned long perfLast=0, perfSum=0, perfCount=0, perfReport=0;

const char* ctrlName(int c){
  switch(c){
    case CTRL_PID:   return "PID";
    case CTRL_LQR:   return "LQR";
    case CTRL_FUZZY: return "FUZZY";
    case CTRL_MPC:   return "MPC";
  }
  return "?";
}

void setup(){
  Serial.begin(115200);
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);

  pinMode(EN1,OUTPUT); pinMode(EN2,OUTPUT); pinMode(EN3,OUTPUT);
  digitalWrite(EN1,HIGH); digitalWrite(EN2,HIGH); digitalWrite(EN3,HIGH);

  delay(1000);          

  digitalWrite(EN1,LOW); digitalWrite(EN2,LOW); digitalWrite(EN3,LOW);

  shapeT0 = millis();
  reportMode(); reportGains(); reportTrim(); reportShape(); reportFf(); reportTracking();
}

void loop(){
  pollCommands();

  double sx = 0, sy = 0;
  trajectory(sx, sy);
  controlStep(sx, sy);
}

void smoothBlend(double u, double duration,
                 double &s, double &sd, double &sdd){
  u = constrain(u, 0.0, 1.0);
  double u2=u*u, u3=u2*u, u4=u3*u, u5=u4*u;
  s = 10.0*u3 - 15.0*u4 + 6.0*u5;
  if(duration <= 0.001){ sd=0; sdd=0; return; }
  sd  = (30.0*u2 - 60.0*u3 + 30.0*u4) / duration;
  sdd = (60.0*u - 180.0*u2 + 120.0*u3) / (duration*duration);
}

void polygonReference(const double vertices[][2], int count,
                      double t, double period,
                      double &x, double &y,
                      double &vx, double &vy,
                      double &ax, double &ay){
  double cycle = fmod(t / period, 1.0);
  double q = cycle * count;
  int from = (int)q;
  if(from >= count) from = 0;
  int to = (from + 1) % count;
  double local = q - floor(q);
  double edgeTime = period / count;
  double s, sd, sdd;
  smoothBlend(local, edgeTime, s, sd, sdd);
  double dx = vertices[to][0] - vertices[from][0];
  double dy = vertices[to][1] - vertices[from][1];
  x  = vertices[from][0] + dx*s;
  y  = vertices[from][1] + dy*s;
  vx = dx*sd;  vy = dy*sd;
  ax = dx*sdd; ay = dy*sdd;
}

bool waypointShape(int s){
  return s==SH_SQUARE || s==SH_TRIANGLE || s==SH_LINE;
}

int waypointCount(int s){
  if(s==SH_SQUARE) return 4;
  if(s==SH_TRIANGLE) return 3;
  if(s==SH_LINE) return 2;
  return 0;
}

double trackingErrorCounts(){
  double dx=(Xoffset+lastSetpoint[0])-filteredX;
  double dy=(Yoffset+lastSetpoint[1])-filteredY;
  return sqrt(dx*dx+dy*dy);
}

void resetPathMotion(){
  pathPhase=0;
  pathRateScale=0;
  pathLastUpdate=millis();
  cornerHold=false;
  cornerStableSince=0;
}

void updatePathPhase(bool transitionComplete){
  unsigned long now=millis();
  if(pathLastUpdate==0){ pathLastUpdate=now; return; }
  double dt=(now-pathLastUpdate)/1000.0;
  pathLastUpdate=now;
  dt=constrain(dt,0.0,0.100);

  if(trajectorySuspended || !detected) return;

  if(!transitionComplete){ pathRateScale=0; return; }
  if(cornerHold){
    pathRateScale=0;
    if(now-cornerStableSince>=CORNER_DWELL_MS){
      cornerHold=false;
      cornerStableSince=0;
    }
    return;
  }

  pathRateScale=1.0;
  double increment=dt/shapePeriod;

  int count=waypointCount(shape);
  if(count>0){
    double nextBoundary=(floor(pathPhase*count+1e-7)+1.0)/count;

    if(pathPhase+increment>=nextBoundary-1e-9){
      pathPhase=(nextBoundary>=1.0) ? 0.0 : nextBoundary;
      cornerHold=true;
      cornerStableSince=now;
      pathRateScale=0;
    } else {
      pathPhase+=increment;
    }
  } else {
    pathPhase=fmod(pathPhase+increment,1.0);
  }
}

void trajectory(double &sx, double &sy){

  if(trajectorySuspended){
    sx=0; sy=0;
    lastSetpoint[0]=lastSetpoint[1]=0;
    referenceVelocity[0]=referenceVelocity[1]=0;
    referenceAccel[0]=referenceAccel[1]=0;
    ffTilt[0]=ffTilt[1]=0;
    return;
  }

  double t = (millis() - shapeT0) / 1000.0;
  bool transitionComplete=(shapeRamp<=0.01 || t>=shapeRamp);
  updatePathPhase(transitionComplete);
  double effectiveOmega=(TWO_PI/shapePeriod)*pathRateScale;
  double a=TWO_PI*pathPhase;

  double bx=0, by=0, bvx=0, bvy=0, bax=0, bay=0;

  switch(shape){
    case SH_CIRCLE:
      bx=cos(a); by=sin(a);
      bvx=-effectiveOmega*sin(a); bvy=effectiveOmega*cos(a);
      bax=-effectiveOmega*effectiveOmega*cos(a);
      bay=-effectiveOmega*effectiveOmega*sin(a);
      break;

    case SH_FIGURE8:
      bx=sin(a); by=0.5*sin(2.0*a);
      bvx=effectiveOmega*cos(a); bvy=effectiveOmega*cos(2.0*a);
      bax=-effectiveOmega*effectiveOmega*sin(a);
      bay=-2.0*effectiveOmega*effectiveOmega*sin(2.0*a);
      break;

    case SH_SQUARE: {
      static const double square[4][2] = {
        {-1,-1}, {1,-1}, {1,1}, {-1,1}
      };
      polygonReference(square,4,pathPhase*shapePeriod,shapePeriod,
                       bx,by,bvx,bvy,bax,bay);
      bvx*=pathRateScale; bvy*=pathRateScale;
      bax*=pathRateScale*pathRateScale;
      bay*=pathRateScale*pathRateScale;
      break;
    }

    case SH_TRIANGLE: {
      static const double triangle[3][2] = {
        {0,-1}, {0.8660254,0.5}, {-0.8660254,0.5}
      };
      polygonReference(triangle,3,pathPhase*shapePeriod,shapePeriod,
                       bx,by,bvx,bvy,bax,bay);
      bvx*=pathRateScale; bvy*=pathRateScale;
      bax*=pathRateScale*pathRateScale;
      bay*=pathRateScale*pathRateScale;
      break;
    }

    case SH_LINE: {
      static const double line[2][2] = {
        {-1,0}, {1,0}
      };
      polygonReference(line,2,pathPhase*shapePeriod,shapePeriod,
                       bx,by,bvx,bvy,bax,bay);
      bvx*=pathRateScale; bvy*=pathRateScale;
      bax*=pathRateScale*pathRateScale;
      bay*=pathRateScale*pathRateScale;
      break;
    }

    case SH_HEART: {
      double st=sin(a), ct=cos(a);
      bx = st*st*st;
      by = (13.0*ct - 5.0*cos(2*a) - 2.0*cos(3*a) - cos(4*a))/17.0;
      bvx = 3.0*st*st*ct*effectiveOmega;
      bvy = (-13.0*st + 10.0*sin(2*a) + 6.0*sin(3*a)
             + 4.0*sin(4*a))*effectiveOmega/17.0;
      bax = (6.0*st*ct*ct - 3.0*st*st*st)
           *effectiveOmega*effectiveOmega;
      bay = (-13.0*ct + 20.0*cos(2*a) + 18.0*cos(3*a)
             + 16.0*cos(4*a))*effectiveOmega*effectiveOmega/17.0;
      break;
    }

    case SH_OFF:
    default:
      break;
  }

  double blend, blendVel, blendAccel;
  if(shapeRamp <= 0.01){
    blend=1.0; blendVel=0; blendAccel=0;
  } else {
    smoothBlend(t/shapeRamp, shapeRamp, blend, blendVel, blendAccel);
  }

  double base[2] = { shapeSize*bx, shapeSize*by };
  double baseVel[2] = { shapeSize*bvx, shapeSize*bvy };
  double baseAccel[2] = { shapeSize*bax, shapeSize*bay };
  if(shape==SH_POINT){
    base[0]=pointTarget[0]; base[1]=pointTarget[1];
    baseVel[0]=baseVel[1]=0;
    baseAccel[0]=baseAccel[1]=0;
  }
  double start[2] = { trajectoryStart[0], trajectoryStart[1] };

  double sp[2], sv[2], sa[2];
  for(int i=0;i<2;i++){
    double delta = base[i] - start[i];
    sp[i] = start[i] + blend*delta;
    sv[i] = blendVel*delta + blend*baseVel[i];
    sa[i] = blendAccel*delta + 2.0*blendVel*baseVel[i]
          + blend*baseAccel[i];
    lastSetpoint[i] = sp[i];
    referenceVelocity[i] = sv[i];
    referenceAccel[i] = sa[i];

    double countToM = (i==0) ? X_COUNT_TO_M : Y_COUNT_TO_M;
    double wantedTilt = (sa[i] * countToM) / PLANT_K;
    ffTilt[i] = ffAlpha*wantedTilt + (1.0-ffAlpha)*ffTilt[i];
    ffTilt[i] = constrain(ffTilt[i], -ffClamp, ffClamp);
  }

  sx=sp[0]; sy=sp[1];
  if(shape==SH_OFF && blend>=1.0){
    ffTilt[0]=0; ffTilt[1]=0;
  }
}

void pollCommands(){
  static char buf[80];
  static uint8_t n = 0;
  static unsigned long lastByteAt = 0;
  while(Serial.available()){
    char c = Serial.read();
    if(c=='\n' || c=='\r'){ if(n>0){ buf[n]='\0'; handleCommand(buf); n=0; } }
    else if(n < sizeof(buf)-1){ buf[n++] = c; lastByteAt=millis(); }
  }

  if(n>0 && millis()-lastByteAt>=25){
    buf[n]='\0'; handleCommand(buf); n=0;
  }
}

bool parseDoubleToken(const char* token, double &value){
  if(token==NULL || token[0]=='\0') return false;
  char* end=NULL;
  value=strtod(token,&end);
  return end!=token && *end=='\0' && !isnan(value) && !isinf(value);
}

bool parseIntToken(const char* token, int &value){
  if(token==NULL || token[0]=='\0') return false;
  char* end=NULL;
  long parsed=strtol(token,&end,10);
  if(end==token || *end!='\0') return false;
  value=(int)parsed;
  return true;
}

void commandError(const char* reason){
  Serial.print("ERR,"); Serial.println(reason);
}

void handleCommand(char* cmd){
  char* context=NULL;
  char* op=strtok_r(cmd,",",&context);
  if(op==NULL) return;

  if(!strcasecmp(op,"C")){
    char* nm=strtok_r(NULL,",",&context);
    int ch=activeCtrl;
    if      (nm && !strcasecmp(nm,"PID"))   ch=CTRL_PID;
    else if (nm && !strcasecmp(nm,"LQR"))   ch=CTRL_LQR;
    else if (nm && !strcasecmp(nm,"FUZZY")) ch=CTRL_FUZZY;
    else if (nm && !strcasecmp(nm,"MPC"))   ch=CTRL_MPC;
    else { commandError("CONTROLLER"); return; }
    activeCtrl=ch; wipeMemory(); reportMode(); return;
  }

  if(!strcasecmp(op,"G")){
    double a,b,c;
    if(!parseDoubleToken(strtok_r(NULL,",",&context),a) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),b) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),c)){
      commandError("GAIN"); return;
    }
    kp=a; ki=b; kd=c; wipeMemory(); reportGains(); return;
  }

  if(!strcasecmp(op,"K")){
    double a,b;
    if(!parseDoubleToken(strtok_r(NULL,",",&context),a) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),b)){
      commandError("LQR"); return;
    }
    K1=a; K2=b; wipeMemory();
    Serial.print("LQRK,"); Serial.print(K1,8);
    Serial.print(","); Serial.println(K2,8); return;
  }

  if(!strcasecmp(op,"F")){
    double a,b,c;
    if(!parseDoubleToken(strtok_r(NULL,",",&context),a) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),b) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),c)){
      commandError("FUZZY"); return;
    }
    fzEmax=max(1.0,a); fzDmax=max(0.1,b);
    fzUmax=constrain(c,0.01,OUT_CLAMP); wipeMemory();
    Serial.print("FUZZ,"); Serial.print(fzEmax,2); Serial.print(",");
    Serial.print(fzDmax,3); Serial.print(","); Serial.println(fzUmax,4); return;
  }

  if(!strcasecmp(op,"M")){
    double a,b,c; int nn;
    if(!parseDoubleToken(strtok_r(NULL,",",&context),a) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),b) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),c) ||
       !parseIntToken(strtok_r(NULL,",",&context),nn)){
      commandError("MPC"); return;
    }
    mpcQ1=a; mpcQ2=b; mpcR=max(1e-6,c); mpcN=constrain(nn,2,40);
    wipeMemory();
    Serial.print("MPCP,"); Serial.print(mpcQ1,4); Serial.print(",");
    Serial.print(mpcQ2,4); Serial.print(","); Serial.print(mpcR,2);
    Serial.print(","); Serial.println(mpcN); return;
  }

  if(!strcasecmp(op,"I")){
    double a;
    if(!parseDoubleToken(strtok_r(NULL,",",&context),a)){
      commandError("ITRIM"); return;
    }
    kiTrim=a; wipeMemory(); reportTrim(); return;
  }

  if(!strcasecmp(op,"P")){
    char* nm=strtok_r(NULL,",",&context);
    int requested=shape;
    if      (nm && !strcasecmp(nm,"OFF"))      requested=SH_OFF;
    else if (nm && !strcasecmp(nm,"CIRCLE"))   requested=SH_CIRCLE;
    else if (nm && !strcasecmp(nm,"SQUARE"))   requested=SH_SQUARE;
    else if (nm && !strcasecmp(nm,"TRIANGLE")) requested=SH_TRIANGLE;
    else if (nm && !strcasecmp(nm,"FIGURE8"))  requested=SH_FIGURE8;
    else if (nm && !strcasecmp(nm,"HEART"))    requested=SH_HEART;
    else if (nm && !strcasecmp(nm,"LINE"))     requested=SH_LINE;
    else { commandError("SHAPE"); return; }

    double sz=shapeSize, pr=shapePeriod, rp=shapeRamp, parsed;
    char* token=strtok_r(NULL,",",&context);
    if(token){ if(!parseDoubleToken(token,parsed)){ commandError("SIZE"); return; } sz=parsed; }
    token=strtok_r(NULL,",",&context);
    if(token){ if(!parseDoubleToken(token,parsed)){ commandError("PERIOD"); return; } pr=parsed; }
    token=strtok_r(NULL,",",&context);
    if(token){ if(!parseDoubleToken(token,parsed)){ commandError("RAMP"); return; } rp=parsed; }

    trajectoryStart[0]=lastSetpoint[0];
    trajectoryStart[1]=lastSetpoint[1];
    shape=requested;
    shapeSize=constrain(sz,10.0,300.0);
    shapePeriod=constrain(pr,1.5,120.0);
    shapeRamp=constrain(rp,0.0,20.0);
    shapeT0=millis();
    resetPathMotion();

    reportShape(); return;
  }

  if(!strcasecmp(op,"R")){
    double absoluteX,absoluteY;
    if(!parseDoubleToken(strtok_r(NULL,",",&context),absoluteX) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),absoluteY)){
      commandError("POINT"); return;
    }
    trajectoryStart[0]=lastSetpoint[0];
    trajectoryStart[1]=lastSetpoint[1];
    pointTarget[0]=constrain(absoluteX-Xoffset,-300.0,300.0);
    pointTarget[1]=constrain(absoluteY-Yoffset,-300.0,300.0);
    shape=SH_POINT;
    shapeRamp=0.65;
    shapeT0=millis();
    resetPathMotion();
    reportShape(); return;
  }

  if(!strcasecmp(op,"W")){
    double a;
    if(!parseDoubleToken(strtok_r(NULL,",",&context),a)){
      commandError("FFWD"); return;
    }
    ffGain=constrain(a,0.0,3.0); reportFf(); return;
  }

  if(!strcasecmp(op,"B")){
    double a,b;
    if(!parseDoubleToken(strtok_r(NULL,",",&context),a) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),b)){
      commandError("BIAS"); return;
    }
    tiltBiasX=constrain(a,-0.10,0.10);
    tiltBiasY=constrain(b,-0.10,0.10);
    wipeMemory();
    Serial.print("BIAS,"); Serial.print(tiltBiasX,5);
    Serial.print(","); Serial.println(tiltBiasY,5); return;
  }

  if(!strcasecmp(op,"T")){
    double a,b,sec=1.5,parsed;
    if(!parseDoubleToken(strtok_r(NULL,",",&context),a) ||
       !parseDoubleToken(strtok_r(NULL,",",&context),b)){
      commandError("TILT"); return;
    }
    char* token=strtok_r(NULL,",",&context);
    if(token){ if(!parseDoubleToken(token,parsed)){ commandError("TILT_TIME"); return; } sec=parsed; }
    manualNx=constrain(a,-OUT_CLAMP,OUT_CLAMP);
    manualNy=constrain(b,-OUT_CLAMP,OUT_CLAMP);
    sec=constrain(sec,0.2,5.0);
    manualTilt=(manualNx!=0 || manualNy!=0);
    manualUntil=millis()+(unsigned long)(sec*1000);
    wipeMemory();
    Serial.print("TILT,"); Serial.print(manualNx,4); Serial.print(",");
    Serial.print(manualNy,4); Serial.print(","); Serial.println(sec,2); return;
  }

  if(!strcasecmp(op,"S")){
    reportMode(); reportGains(); reportTrim(); reportShape(); reportFf();
    reportTracking(); return;
  }

  commandError("COMMAND");
}

void reportMode(){
  Serial.print("MODE,"); Serial.print(ctrlName(activeCtrl));
  Serial.print(","); Serial.println(ctrlReady[activeCtrl] ? "READY" : "STUB");
}
void reportGains(){
  Serial.print("GAIN,"); Serial.print(kp,10); Serial.print(",");
  Serial.print(ki,10); Serial.print(","); Serial.println(kd,10);
}
void reportTrim(){
  Serial.print("ITRIM,"); Serial.println(kiTrim,10);
}
void reportShape(){
  Serial.print("SHAPE,"); Serial.print(shapeName(shape)); Serial.print(",");
  if(shape==SH_POINT){
    Serial.print(Xoffset+pointTarget[0],1); Serial.print(",");
    Serial.println(Yoffset+pointTarget[1],1);
  } else {
    Serial.print(shapeSize,1); Serial.print(","); Serial.println(shapePeriod,2);
  }
}
void reportFf(){
  Serial.print("FFWD,"); Serial.println(ffGain,3);
}

void reportTracking(){
  Serial.print("TRACK,");
  Serial.print(detected ? "DETECTED" : "NO_BALL"); Serial.print(",");
  Serial.print(trajectorySuspended ? "PAUSED" : "RUNNING"); Serial.print(",");
  Serial.print(shapeName(shape)); Serial.print(",");
  Serial.print(lastSetpoint[0],2); Serial.print(",");
  Serial.print(lastSetpoint[1],2); Serial.print(",");
  Serial.print(trackingErrorCounts(),2); Serial.print(",");
  Serial.print(pathRateScale,2); Serial.print(",");
  Serial.println(cornerHold ? "HOLD" : "MOVE");
}

void wipeMemory(){
  for(int i=0;i<2;i++){
    error[i]=0; errorPrev[i]=0; integr[i]=0; deriv[i]=0; out[i]=0;
  }
}

double axisPID(int i){
  return kp*error[i] + ki*integr[i] + kd*deriv[i];
}

double axisLQR(int i){
  return K1*error[i] + K2*deriv[i] + kiTrim*integr[i];
}

double triMF(double x, double centre, double halfWidth){
  double d = fabs(x - centre);
  if(d >= halfWidth) return 0.0;
  return 1.0 - d/halfWidth;
}

const double FZ_CENTRE[5] = { -1.0, -0.5, 0.0, 0.5, 1.0 };

const uint8_t FZ_RULES[5][5] = {
  { 0, 0, 0, 1, 2 },
  { 0, 0, 1, 2, 3 },
  { 0, 1, 2, 3, 4 },
  { 1, 2, 3, 4, 4 },
  { 2, 3, 4, 4, 4 }
};

double axisFuzzy(int i){
  double e  = constrain(error[i] / fzEmax, -1.0, 1.0);
  double de = constrain(deriv[i] / fzDmax, -1.0, 1.0);

  double muE[5], muD[5];
  for(int k=0;k<5;k++){
    muE[k] = triMF(e,  FZ_CENTRE[k], 0.5);
    muD[k] = triMF(de, FZ_CENTRE[k], 0.5);
  }

  double agg[5] = {0,0,0,0,0};
  for(int a=0;a<5;a++){
    if(muE[a] <= 0) continue;
    for(int b=0;b<5;b++){
      if(muD[b] <= 0) continue;
      double strength = min(muE[a], muD[b]);
      uint8_t o = FZ_RULES[a][b];
      if(strength > agg[o]) agg[o] = strength;
    }
  }

  const int STEPS = 41;
  double numer = 0, denom = 0;
  for(int s=0;s<STEPS;s++){
    double u = -1.0 + 2.0 * s / (STEPS - 1);
    double mu = 0;
    for(int k=0;k<5;k++){
      if(agg[k] <= 0) continue;
      double m = min(agg[k], triMF(u, FZ_CENTRE[k], 0.5));
      if(m > mu) mu = m;
    }
    numer += u * mu;
    denom += mu;
  }
  double fuzzyOut = (denom < 1e-9) ? 0.0 : (numer / denom) * fzUmax;

  return fuzzyOut + kiTrim*integr[i];
}

double axisMPC(int i){
  double p0 = error[i];
  double v0 = deriv[i] / mpcDt;

  double num = 0.0;
  double den = mpcN * mpcR;

  for(int k=1;k<=mpcN;k++){
    double tk = k * mpcDt;
    double dp = 0.5 * plantB * tk * tk;
    double dv = plantB * tk;
    double pk = p0 + v0 * tk;
    double vk = v0;
    num += mpcQ1 * pk * dp + mpcQ2 * vk * dv;
    den += mpcQ1 * dp * dp + mpcQ2 * dv * dv;
  }

  if(fabs(den) < 1e-12) return 0.0;

  double u = -num / den;
  return constrain(u + kiTrim*integr[i], -OUT_CLAMP, OUT_CLAMP);
}

double computeAxis(int i){
  switch(activeCtrl){
    case CTRL_PID:   return axisPID(i);
    case CTRL_LQR:   return axisLQR(i);
    case CTRL_FUZZY: return axisFuzzy(i);
    case CTRL_MPC:   return axisMPC(i);
  }
  return 0.0;
}

void suspendTrajectoryAfterBallLoss(){
  trajectorySuspended=true;
  pathRateScale=0;
  pathLastUpdate=millis();
  trajectoryStart[0]=trajectoryStart[1]=0;
  lastSetpoint[0]=lastSetpoint[1]=0;
  referenceVelocity[0]=referenceVelocity[1]=0;
  referenceAccel[0]=referenceAccel[1]=0;
  ffTilt[0]=ffTilt[1]=0;
}

void resumeTrajectoryAfterBallReturn(){
  trajectorySuspended=false;
  trajectoryStart[0]=trajectoryStart[1]=0;
  lastSetpoint[0]=lastSetpoint[1]=0;
  referenceVelocity[0]=referenceVelocity[1]=0;
  referenceAccel[0]=referenceAccel[1]=0;
  ffTilt[0]=ffTilt[1]=0;
  shapeT0=millis();
  resetPathMotion();
  wipeMemory();
  Serial.print("STATE,BALL_DETECTED,"); Serial.println(shapeName(shape));
}

void controlStep(double setpointX, double setpointY){
  TSPoint p = ts.getPoint();

  unsigned long nowMs = millis();
  if(perfLast != 0){
    unsigned long d = nowMs - perfLast;
    perfSum += d; perfCount++;
  }
  perfLast = nowMs;
  if(nowMs - perfReport > 1000 && perfCount > 0){
    Serial.print("PERF,");
    Serial.println((double)perfSum / perfCount, 2);
    perfSum = 0; perfCount = 0; perfReport = nowMs;
  }

  if(p.x != 0){
    detected = 1;
    lastBallSeen = nowMs;
    if(trajectorySuspended) resumeTrajectoryAfterBallReturn();

    filteredX = (filterAlpha * p.x) + ((1.0 - filterAlpha) * filteredX);
    filteredY = (filterAlpha * p.y) + ((1.0 - filterAlpha) * filteredY);

    for(int i=0;i<2;i++){
      errorPrev[i] = error[i];

      error[i] = (i==0) * ((Xoffset + setpointX) - filteredX)
               + (i==1) * ((Yoffset + setpointY) - filteredY);

      integr[i] += error[i] + errorPrev[i];
      integr[i] = constrain(integr[i], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

      deriv[i] = error[i]-errorPrev[i];
      if(isnan(deriv[i]) || isinf(deriv[i])) deriv[i] = 0;

      out[i] = manualTilt ? 0.0 : computeAxis(i);
      out[i] = constrain(out[i], -OUT_CLAMP, OUT_CLAMP);
    }

    for(int i=0;i<3;i++){
      speedPrev[i]=speed[i];
      long current=(i==A)*stepperA.currentPosition()
                  +(i==B)*stepperB.currentPosition()
                  +(i==C)*stepperC.currentPosition();
      double requestedSpeed=abs(current-pos[i])*ks;
      speed[i]=constrain(requestedSpeed,
                         max(0.0,speedPrev[i]-SPEED_SLEW),
                         speedPrev[i]+SPEED_SLEW);
      speed[i]=constrain(speed[i],0.0,MAX_TRACK_SPEED);
    }

    static unsigned long lastPrint = 0;
    if(millis() - lastPrint > 20){
      Serial.print("DATA,");
      Serial.print(filteredX); Serial.print(",");
      Serial.print(filteredY); Serial.print(",");
      Serial.print(Xoffset + setpointX); Serial.print(",");
      Serial.print(Yoffset + setpointY); Serial.print(",");
      Serial.println(ctrlName(activeCtrl));
      lastPrint = millis();
    }
  }
  else if(detected && nowMs-lastBallSeen>=BALL_LOSS_TIMEOUT_MS){
    detected = 0;
    filteredX = Xoffset; filteredY = Yoffset;
    suspendTrajectoryAfterBallLoss();
    wipeMemory();
    Serial.print("STATE,BALL_LOST,"); Serial.println(shapeName(shape));
  }

  if(manualTilt && (long)(millis()-manualUntil) >= 0){
    manualTilt=false; wipeMemory();
    Serial.println("TILT,END");
  }

  double tiltX,tiltY;
  if(manualTilt){
    tiltX=manualNx; tiltY=manualNy;
  } else {
    double rawX=out[0]+ffGain*ffTilt[0]+tiltBiasX;
    double rawY=out[1]+ffGain*ffTilt[1]+tiltBiasY;
    tiltX=rawX*cos(alignAngle)-rawY*sin(alignAngle);
    tiltY=rawX*sin(alignAngle)+rawY*cos(alignAngle);
  }

  moveTo(4.25,tiltX,tiltY);

  timeI=millis();
  while(millis()-timeI<20){
    pollCommands();
    stepperA.run(); stepperB.run(); stepperC.run();
  }
}

void moveTo(double hz, double nx, double ny){
  bool followTilt=detected || manualTilt;
  if(!followTilt){ nx=0; ny=0; }

  long nextPos[3];
  for(int i=0;i<3;i++){
    double theta=machine.theta(i,hz,nx,ny);
    if(isnan(theta) || isinf(theta)){
      static unsigned long lastIkError=0;
      if(millis()-lastIkError>500){
        Serial.println("ERR,IK_RANGE");
        lastIkError=millis();
      }
      return; 
    }
    nextPos[i]=round((angOrig-theta)*angToStep);
  }

  for(int i=0;i<3;i++) pos[i]=nextPos[i];

  if(followTilt){
    stepperA.setMaxSpeed(speed[A]);
    stepperB.setMaxSpeed(speed[B]);
    stepperC.setMaxSpeed(speed[C]);
    stepperA.setAcceleration(speed[A]*30);
    stepperB.setAcceleration(speed[B]*30);
    stepperC.setAcceleration(speed[C]*30);

    stepperA.moveTo(pos[A]); stepperB.moveTo(pos[B]); stepperC.moveTo(pos[C]);
  } else {
    stepperA.setMaxSpeed(800);
    stepperB.setMaxSpeed(800);
    stepperC.setMaxSpeed(800);
    stepperA.setAcceleration(12000);
    stepperB.setAcceleration(12000);
    stepperC.setAcceleration(12000);
    stepperA.moveTo(pos[A]); stepperB.moveTo(pos[B]); stepperC.moveTo(pos[C]);
  }
}
