#include <Arduino.h>
#include "Controladores.h"

// Metodos privados
int Controladores::Limites_fisicos(float Control)
{
  if (Control > L_max)
  {
    return L_max;
  }
  else if (Control < L_min)
  {
    return L_min;
  }
  else
  {
    return (int)(Control);
  }
}
float Controladores::AjusteSobreAcumulacionIntegral(float I)
{
  if ((I <= SAIMAX)&&(I >= SAIMIN))
  {
    return I;
  } else
  {
    if (I > SAIMAX)
    {
      return SAIMAX;
    }
    else if (I < SAIMIN)
    {
      return SAIMIN;
    }
  }
}

// Metodos publicos
void Controladores::Constants(float Cp, float Ci, float Cd,float Time)
{
  Kp = Cp;
  Ki = Ci;
  Kd = Cd;
  T = Time;
}
// Algoritmo Proporcional ***********************************
int Controladores::Get_P(float Consigna, float FeedBack)
{
  // Calculo del error **********************
  error = Consigna - FeedBack;

  // Calculo del control proporcional **********
  float P = error * Kp;

  // Evita que se sobrepase los límites físicos de los actuadores ******
  Ultimo_control = Limites_fisicos(P);
  return (int)Ultimo_control;
}
// Algoritmo PI ***********************************
int Controladores::Get_PI(float Consigna, float FeedBack)
{
  // Calculo del error **********************
  error = Consigna - FeedBack;

  // Calculo del control proporcional **********
  float P = error * Kp;

  // Calculo del control integral **********
  float I = T * (Kp / Ki) * error + Ultimo_control;
  I = AjusteSobreAcumulacionIntegral(I);

  //PID ***************
  float u_PI = P + I;

  // Evita que se sobrepase los límites físicos de los actuadores ******
  Ultimo_control = Limites_fisicos(u_PI);
  return (int)Ultimo_control;
}
// Algoritmo PID ***********************************
int Controladores::Get_PID(float Consigna, float FeedBack)
{
  // Calculo del error **********************
  error_pasado = error;
  error = Consigna - FeedBack;

  // Calculo del control proporcional **********
  float P = error * Kp;

  // Calculo del control integral **********
  float I = T * (Kp / Ki) * error + Ultimo_control;
  I = AjusteSobreAcumulacionIntegral(I);

  // Calculo del control derivativo **********
  float D = Kd * (Kp / T) * (error_pasado - error);

  //PID ***************
  float PID = P + I + D;


  // Evita que se sobrepase los límites físicos de los actuadores ******
  Ultimo_control = Limites_fisicos(PID);
  return (int)Ultimo_control;
}
