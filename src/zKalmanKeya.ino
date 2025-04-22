//KALMAN
//float R   = 1;              // varianza del rumore sulla misura dell'angolo stimato
//float Q   = 1e-04;          // varianza del disturbo sul processo
float Pp  = 0.0;            // P(t|t-1) varianza dell'errore di predizione
float K   = 0.0;            // Kalman gain
float P   = 1.0;            // P(t|t) varianza dell'errore di filtraggio
float Xp  = 0.0;            // x_^(t|t-1) predizione dello stato precedente
float X   = 0.0;            // x_^(t|t) stato filtrato

float heading_rate = 0;
float headingOld = 0;
float varianceMean;
float angleVariance;
float heading;

uint16_t lenVarianceBuffer = settings.secondsVarianceBuffer / settings.intervalINS;
float varianceBuffer[200] = {0.0f};
uint16_t indexVarBuffer = 0;

void angleStimeUpdate(){
  if(usingUM982)
    heading = atof(umHeading);
  else
    heading = atof(insHeading);
  heading_rate = (heading - headingOld)/settings.intervalINS;
  headingOld = heading;
  if(heading_rate>300)
    heading_rate-=360;
  if(heading_rate<-300)
    heading_rate+=360;
  insWheelAngle = atan(heading_rate/RAD_TO_DEG*calibrationData.wheelBase/speed) * RAD_TO_DEG * workingDir;
  if(!(insWheelAngle<50 && insWheelAngle>-50))
    insWheelAngle=0;
  //dualWheelAngleWT61 = atan(headingRateWT/RAD_TO_DEG*calibrationData.wheelBase/speed*-1) * RAD_TO_DEG * workingDir;

  //update kalman measure variance R
  if(speed > (settings.minSpeedKalman*0.7) && abs(insWheelAngle)<30 && UM981_aligned){
    varianceBuffer[indexVarBuffer++] = insWheelAngle;
    indexVarBuffer = indexVarBuffer % lenVarianceBuffer;

    //compute mean
    varianceMean = 0;
    for (uint16_t i = 0; i < lenVarianceBuffer; i++)
      varianceMean += varianceBuffer[i];
    varianceMean /= lenVarianceBuffer;

    // Compute variance
    angleVariance = 0;
    for (int i = 0; i < lenVarianceBuffer; i++) {
      angleVariance += (varianceBuffer[i] - varianceMean) * (varianceBuffer[i] - varianceMean);
    }
    angleVariance /= (lenVarianceBuffer - 1);

    if (debugState){
      debugPrint("Variance ");
      debugPrintln(angleVariance);
    }
  }
  if (debugState == EXPERIMENT){
      debugPrint("speed ");
      debugPrint(speed);
      debugPrint(" insWheelAngle ");
      debugPrint(insWheelAngle);
      debugPrint(" insStatus ");
      debugPrintln(insStatus);
    }

  if(speed>settings.minSpeedKalman && UM981_aligned)
    KalmanUpdate();
}

void KalmanUpdate(){
  float angleDiff = (keyaEncoder /  steerSettings.keyaSteerSensorCounts) - steerAngleActualOld;              //how the wheelAngle changed according to encoder from last kalman update

  X = KalmanWheelAngle;
  // --- Kalman process ---
  Pp = P + settings.kalmanQ;                       // (PREDICTION) predizione della varianza dell'errore al prossimo step
  Xp = X + angleDiff;                                     // (PREDICTION) predizione dello stato al prossimo step
  K = Pp/(Pp + (settings.kalmanR*angleVariance));  // (CORRECTION) Kalman gain
  P = (1-K)*Pp;                                           // (CORRECTION) aggiornamento della varianza dell'errore di filtraggio
  X = Xp + K*(insWheelAngle-Xp);                          // (CORRECTION) stima di Kalman dell'output del sensore

  KalmanWheelAngle = X;
  steerAngleActualOld = keyaEncoder /  steerSettings.keyaSteerSensorCounts;
}