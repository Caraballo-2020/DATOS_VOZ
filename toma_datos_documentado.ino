#include <PDM.h> // Libreria para leer el microfono del Arduino
#include <math.h> // Para hacer calculos matematicos como logaritmos y cosenos
#include "arduinoFFT.h" // La libreria para sacar la transformada de Fourier

// --- Configuracion de Audio ---
#define SAMPLE_RATE 16000 // Aca le decimos al microfono que tome 16000 muestras por segundo
#define BUFFER_SIZE 256
#define DOWNSAMPLE_FACTOR 2 // Esto es para bajar la frecuencia a la mitad (8000 Hz) y no saturar el Arduino
#define WINDOW_SIZE 256 // El tamaño de la ventana de audio que vamos a analizar a la vez
#define HOP_SIZE 128 // El avance o solapamiento, osea, corremos la ventana a la mitad para no perder info del sonido
#define WARMUP_TIME 2000 // se definen 2 segundos que esperamos al prender para que el microfono no bote ruido o basura

// --- Configuracion MFCC ---
#define NUM_MFCC 13 // Se define el numero de coeficientes que se calcularan por ventana
#define HISTORY_FRAMES 10   /*Esta variable define cada cuantas ventanas se calcula el promedio de los MFCC (Se calcula el promedio de los MFCC 1, en la siguiente columna el de los MFCC 2 y asi hasta los MFCC 13. Cada tipo de MFCC se calcula aparte, es decir, con su respectivo grupo)*/  

short sampleBuffer[BUFFER_SIZE]; // Aca se guarda el audio crudo que va entrando

volatile int samplesRead = 0; // Contador de cuantas muestras van

// Buffers para la FFT y el procesamiento de señales
double timeBuffer[WINDOW_SIZE];  
double vReal[WINDOW_SIZE]; // Aca van los datos reales para calcular la FFT
double vImag[WINDOW_SIZE]; // Aca van los datos imaginarios (se dejan en 0)
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, WINDOW_SIZE, 8000.0);

// Matrices para ir guardando el historial y sacar los promedios
float mfcc_current[NUM_MFCC];
float mfcc_history[HISTORY_FRAMES][NUM_MFCC];
float mfcc_averages[NUM_MFCC];
int historyIndex = 0;
bool historyFilled = false;

int bufferIndex = 0;

// Esta funcion se ejecuta en segundo plano y va leyendo el microfono constantemente
void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

void setup() {
  Serial.begin(921600); // Velocidad alta para mandar los datos rapido
  while (!Serial);

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, SAMPLE_RATE)) {
    Serial.println("Error iniciando PDM!");
    while (1);
  }
  PDM.setGain(20); // Le subimos un poco la sensibilidad al microfono

  // Aca solo imprimimos los nombres de las 13 columnas una sola vez al arrancar
  for(int i=1; i<=13; i++) { 
    Serial.print("AVG_"); 
    Serial.print(i); 
    if(i<13) Serial.print(","); 
  }
  Serial.println();

  // Llenamos la matriz de historial con ceros para empezar limpios
  for(int i=0; i<HISTORY_FRAMES; i++) {
    for(int j=0; j<NUM_MFCC; j++) mfcc_history[i][j] = 0.0;
  }
}

void loop() {
  if (samplesRead) {
    
    // Ignoramos el audio los primeros 2 segundos para que el microfono se estabilice
    if (millis() < WARMUP_TIME) {
      samplesRead = 0;
      bufferIndex = 0; 
      return; 
    }

    // Procesamiento de la señal
    // Aca hacemos el downsampling, osea agrupamos de a 2 muestras y sacamos un solo promedio
    for (int i = 0; i < samplesRead; i += DOWNSAMPLE_FACTOR) {
      long suma = 0;
      int count = 0;

      for (int j = 0; j < DOWNSAMPLE_FACTOR; j++) {
        if (i + j < samplesRead) {
          suma += sampleBuffer[i + j];
          count++;
        }
      }

      // Vamos metiendo ese resultado al buffer de tiempo
      timeBuffer[bufferIndex] = (double)suma / count;
      bufferIndex++;

      // Cuando ya llenamos el arreglo con los 256 datos de audio, empezamos a calcular todo
      if (bufferIndex == WINDOW_SIZE) {
        
        // Copiamos los datos limpios a las variables que usa la libreria FFT
        for(int k=0; k<WINDOW_SIZE; k++) {
          vReal[k] = timeBuffer[k];
          vImag[k] = 0.0; // Los imaginarios siempre van en 0 aca
        }

        // Aplicamos la ventana Hamming y la FFT para ver que frecuencias tiene ese pedacito de audio
        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
        FFT.compute(FFTDirection::Forward);
        FFT.complexToMagnitude(); 

        // Filtros Mel, esto es para simular matematicamente como escucha el oido humano
        // Basicamente agrupamos las frecuencias en 13 bandas 
        int melBands[NUM_MFCC] = {1, 2, 2, 3, 4, 5, 7, 9, 12, 16, 20, 25, 21}; 
        int currentBin = 1; 
        float logEnergies[NUM_MFCC];

        // Aca calculamos cuanta energia hay en cada banda y le sacamos el logaritmo
        for (int m = 0; m < NUM_MFCC; m++) {
          float bandEnergy = 0.0;
          for (int b = 0; b < melBands[m]; b++) {
            if (currentBin < 128) {
              bandEnergy += vReal[currentBin];
              currentBin++;
            }
          }
          // Se suma 1e-6 para que el logaritmo no se explote o de error si la energia da cero
          logEnergies[m] = log10(bandEnergy + 1e-6);
        }

        // Transformada Discreta del Coseno 
        // Aca aplicamos la formula para comprimir toda esa info en los 13 coeficientes MFCC crudos
        for (int k = 0; k < NUM_MFCC; k++) {
          float sum = 0.0;
          for (int n = 0; n < NUM_MFCC; n++) {
            sum += logEnergies[n] * cos(PI * k * (n + 0.5) / NUM_MFCC);
          }
          mfcc_current[k] = sum;
        }

        // Metemos esos 13 datos recien calculados al historial para luego promediarlos
        for (int k = 0; k < NUM_MFCC; k++) {
          mfcc_history[historyIndex][k] = mfcc_current[k];
        }
        
        historyIndex++;
        if (historyIndex >= HISTORY_FRAMES) {
          historyIndex = 0; // Si llegamos a 10 ventanas, volvemos a empezar para ir borrando las mas viejas
          historyFilled = true;
        }

        // Revisamos cuantos datos tenemos listos para promediar
        int framesToAverage = historyFilled ? HISTORY_FRAMES : historyIndex;
        if (framesToAverage == 0) framesToAverage = 1; 

        // Aca por fin sacamos el promedio de los ultimos 10 calculos para estabilizar los datos y matar el ruido
        for (int k = 0; k < NUM_MFCC; k++) {
          float sumAvg = 0.0;
          for (int h = 0; h < framesToAverage; h++) {
            sumAvg += mfcc_history[h][k];
          }
          mfcc_averages[k] = sumAvg / framesToAverage;
        }

        // Mandamos los 13 promedios ya limpios por el puerto serial, separados por comas
        for(int k = 0; k < NUM_MFCC; k++) {
          Serial.print(mfcc_averages[k], 2);
          if(k < NUM_MFCC - 1) Serial.print(",");
        }
        Serial.println();

        // Solapamiento
        // Corremos el audio hacia atras a la mitad del buffer para que la siguiente ventana se mezcle con el final de esta
        for (int k = 0; k < HOP_SIZE; k++) {
          timeBuffer[k] = timeBuffer[k + HOP_SIZE];
        }
        bufferIndex = HOP_SIZE;
      }
    }
    samplesRead = 0;
  }
}