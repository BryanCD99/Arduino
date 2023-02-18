
// Limite sobrecarga de acción integral ****
const int SAIMAX = 200; //Maximo
const int SAIMIN = 0; //Minimo
// Límites físicos *********************
const int L_max = 200;  //Máx PWM
const int L_min = 0; //Min PWM


class Controladores
{ private:

    float Kp;
    float Ki;
    float Kd;
    float T;

    // Variables PI **********************
    float error;
    float error_pasado;
    float Ultimo_control;

    // Evita que la accion de control se salga de los valores físicos que acepta el actuador *****
    int Limites_fisicos(float Control);

    // Verifica que no se acumule demasiada accion integral **********
    float AjusteSobreAcumulacionIntegral(float I);

  public:
    // Constructor de la clase
    Controladores() {
      Kp = 1;
      Ki = 1;
      Kd = 0;
      T = 1;
      error = 0;
      Ultimo_control = 0;
    }
    // Funcion de ajuste de constantes del controlador
    void Constants(float Cp, float Ci, float Cd, float Time); // Configura nuevas constantes para el controlador
    
    // Algoritmo Proporcional ***********************************
    int Get_P(float Consigna, float FeedBack);              // Calcula el P en base al SetPoint y el Feedback
    // Algoritmo PI ***********************************
    int Get_PI(float Consigna, float FeedBack);              // Calcula el PI en base al SetPoint y el Feedback
    // Algoritmo PID ***********************************
    int Get_PID(float Consigna, float FeedBack);
};
