#include<TinyWireS.h>
//#include <SendOnlySoftwareSerial.h>
int n = 0;
int pasos1 = 0;
int pasos2 = 0;
String DireccionString = "00000000";

//--------------------------CAMBIAR AL REPROGRAMAR---------------------//
int ATTinyNumber = 3;
//--------------------------CAMBIAR AL REPROGRAMAR---------------------//

unsigned long lastMove = 0;
int resetFlag = 0;
int byteCounter = 1;

byte byte1, byte2, byte3, byte4, byte5, byte6;

//const int Tx = 7; //SoftwareSerial TX

//SendOnlySoftwareSerial SWSerial(Tx);

double mimir = 0.0;

double divisor = 0.05;
double delta = 0.2;
double maxVel = 4.0;

byte ACK1, ACK2, ACK3, ACK4, ACK5, ACKFINISH, I2C_ADDRESS;

byte corruptData = 0b11111111;

void receiveData2(int bytecount)
{

  switch (byteCounter)
  {
    case 1:
      {
        byte1 = TinyWireS.read();
        if (byte1 != corruptData)
        {
          byteCounter++;
        }
        else
        {
          byteCounter = 1;
        }
        //SWSerial.print("byte1: ");
        //SWSerial.println(byte1);
        break;
      }
    case 2:
      {
        byte2 = TinyWireS.read();
        if (byte2 != corruptData)
        {
          byteCounter++;
        }
        else
        {
          byteCounter = 1;
        }
        //SWSerial.print("byte2: ");
        //SWSerial.println(byte2);
        break;
      }
    case 3:
      {
        byte3 = TinyWireS.read();
        if (byte3 != corruptData)
        {
          byteCounter++;
        }
        else
        {
          byteCounter = 1;
        }
        //SWSerial.print("byte3: ");
        //SWSerial.println(byte3);
        break;
      }
    case 4:
      {
        byte4 = TinyWireS.read();
        if (byte4 != corruptData)
        {
          byteCounter++;
        }
        else
        {
          byteCounter = 1;
        }
        //SWSerial.print("byte4: ");
        //SWSerial.println(byte4);
        break;
      }
    default:
      {
        break;
      }

  }
}

void customDelay(int pasos_totales, int paso_actual) {
  /*
    customDelay()
      retorna un entero que corresponde al numero de milisegundos entre envios a los motores
      Inputs
        pasos_totales: el numero de pasos a dar, maximo entre los pasos del motor uno y dos
        paso_actual: el paso en el que va la funcion organizadorDeBytes()
  */
  //
  //  if (paso_actual == 0) {
  //    mimir = maxVel + pasos_totales * divisor * delta;
  //  } else if (paso_actual > 0 and paso_actual < pasos_totales * divisor) {
  //    mimir = mimir - delta;
  //  } else if ( paso_actual >= pasos_totales * (1 - divisor)) {
  //    mimir = mimir + delta;
  //  }


  if (paso_actual == 0) {
    mimir = maxVel;
  } else if ( paso_actual >= pasos_totales * (1 - divisor)) {
    mimir = mimir + delta;
  }

}

void organizadorDeBytes(String direccion, int steps1, int steps2) {
  /*
    organizadorDeBytes()
      Recibe info de los motores, organiza los datos y llama a outputMotores() en intervalos customDelay()
        retorna un void
      Inputs
        direccion1/direccion2: Informacion sobre que motor es y la direccion de movimiento
        steps1/steps2: Numero de pasos de cada motor
  */
  //SWSerial.println("Entro Organizador");
  //String prueba = "00110001";
  char enable2 = direccion[7];
  char direc2 = direccion[6];
  char vel2 = direccion[5];
  char enable1 = direccion[3];
  char direc1 = direccion[2];
  char vel1 = direccion[1];


  // 3 step; 1 es 1; 2 es direccion; 11 fijos en la mitad y

  int pasos_mayor = max(pasos1, pasos2);
  char prendido1 = '1';
  char prendido2 = '1';

  //  SWSerial.println(int(enable1 - 48));
  //  SWSerial.println(int(direc1 - 48));
  //  SWSerial.println(int(prendido1 - 48));
  //  SWSerial.println(int(enable2 - 48));
  //  SWSerial.println(int(direc2 - 48));
  //  SWSerial.println(int(prendido2 - 48));

  digitalWrite(0, int(enable1 - 48)); // Enable1
  digitalWrite(1, int(direc1 - 48)); // Direccion1
  digitalWrite(5, int(direc2 - 48)); // Direccion2 & MISO
  digitalWrite(7, int(enable2 - 48)); // Enable2

  delayMillis(100);

  for (int pasos_dados = 0; pasos_dados < pasos_mayor; pasos_dados++) {

    if (pasos_dados == pasos1) {
      prendido1 = '0';
    }

    if (pasos_dados == pasos2) {
      prendido2 = '0';
    }

    //SWSerial.print("estado1: ");
    //SWSerial.println(int(direc1 - 48));

    digitalWrite(2, LOW); // Pulso1
    digitalWrite(3, LOW); // Pulso2

    //delay(customDelay(pasos_mayor, pasos_dados);
    //SWSerial.print("estado2: ");

    digitalWrite(2, int(prendido1 - 48)); // Pulso1
    digitalWrite(3, int(prendido2 - 48)); // Pulso2

    customDelay(pasos_mayor, pasos_dados);
    delay(mimir);
    //SWSerial.print("pasos: ");
    //SWSerial.println(pasos_dados);
  }
}

void requestEvent()
{
  switch (byteCounter)
  {
    case 1:
      {
        TinyWireS.write(ACKFINISH);
        break;
      }
    case 2:
      {
        TinyWireS.write(ACK1);
        break;
      }
    case 3:
      {
        TinyWireS.write(ACK2);
        break;
      }
    case 4:
      {
        TinyWireS.write(ACK3);
        break;
      }
    case 5:
      {
        TinyWireS.write(ACK4);
        break;
      }
    case 6:
      {
        TinyWireS.write(ACK5);
        break;
      }
    default:
      {
        break;
      }

  }
}

void setup() {
  /*
    setup()
      Recibe los datos y llama a organizadorDeBytes()
  */
  //Direccion depende de pos en caja
  //SWSerial.begin(38400);

  if (ATTinyNumber == 1)
  {
    ACK1 = 0b00100001;
    ACK2 = 0b00100010;
    ACK3 = 0b00100100;
    ACK4 = 0b00101000;
    ACK5 = 0b00110000;

    ACKFINISH = 0b00111111;
    I2C_ADDRESS = 0x23;
  }
  else if (ATTinyNumber == 2)
  {
    ACK1 = 0b01000001;
    ACK2 = 0b01000010;
    ACK3 = 0b01000100;
    ACK4 = 0b01001000;
    ACK5 = 0b01010000;

    ACKFINISH = 0b01011111;
    I2C_ADDRESS = 0x24;
  }
  else if (ATTinyNumber == 3)
  {
    ACK1 = 0b10000001;
    ACK2 = 0b10000010;
    ACK3 = 0b10000100;
    ACK4 = 0b10001000;
    ACK5 = 0b10010000;

    ACKFINISH = 0b10011111;
    I2C_ADDRESS = 0x25;
  }

  TinyWireS.begin(I2C_ADDRESS);       // comienza la comunicación I2C como esclavo con dirección 0x08
  delay(2000);
  TinyWireS.onReceive(receiveData2);  // llama a la función receiveEvent() cuando se reciban datos por I2C
  TinyWireS.onRequest(requestEvent);
  pinMode(0, OUTPUT); // Enable1
  pinMode(1, OUTPUT); // Direccion1
  pinMode(2, OUTPUT); // Pulso1
  pinMode(3, OUTPUT); // Pulso2
  pinMode(5, OUTPUT); // Direccion2 & MISO
  pinMode(7, OUTPUT); // Enable2
  //SWSerial.print("ON");
  digitalWrite(0, HIGH);
  digitalWrite(7, HIGH);
}


void loop() {

  if (millis() - lastMove > 30000 and resetFlag == 0)
  {
    byteCounter = 1;
    resetFlag = 1;
  }

  if (byteCounter == 5)
  {
    byte5 = TinyWireS.read();

    if (byte5 != corruptData)
    {
      byteCounter++;
      delayMillis(100);

      if (TinyWireS.available() <= 0)
      {
        pasos1 = (byte2 << 8) | byte3;
        pasos2 = (byte4 << 8) | byte5;

        DireccionString = "00000000";
        for (int i = 0; i < 8; i++) {
          DireccionString[i] = char(bitRead(byte1, 7 - i) + '0');
        }
        //SWSerial.print(pasos1);
        //SWSerial.print(", ");
        //SWSerial.print(pasos2);
        //SWSerial.print(", ");
        //SWSerial.println(DireccionString);

        organizadorDeBytes(DireccionString, pasos1, pasos2);
        byteCounter = 1;
        lastMove = millis();
        resetFlag = 0;
      }
      else
      {
        byte5 = TinyWireS.read();
        byteCounter = 1;
        lastMove = millis();
        resetFlag = 0;
      }
    }
    else
    {
      byteCounter = 1;
      lastMove = millis();
      resetFlag = 0;
    }
    //SWSerial.print("byte5: ");
    //SWSerial.println(byte5);

    //SWSerial.println("Entered processing");

  }
}

void delayMillis(unsigned long tempo)
{
unsigned long  previousTime = millis();
unsigned long currentTime = previousTime;

  while (currentTime - previousTime < tempo)
  {
    currentTime = millis();
  }

}
