#define TRIG_PIN 10
#define ECHO_PIN 9
#define LED_PIN 7

long duration;
int distance;
int firstDistance = -1;
int lastDistance = -1;
int peopleEntered = 0;
int peopleExited = 0;
int peopleInRoom = 0;
const int thresholdDistance = 200; // Ajuste conforme necessário para a sua instalação
const int targetPeopleCount = 10;
bool objectDetected = false;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Medida da distância atual
  distance = getDistance();

  // Verifica se o objeto está dentro da faixa de detecção
  if (distance > 0 && distance < thresholdDistance) {
    if (!objectDetected) {
      // Se o objeto estava ausente e agora foi detectado
      firstDistance = distance; // Armazena a primeira distância
      objectDetected = true;
    } else {
      // Se o objeto já estava detectado, atualiza o último valor
      lastDistance = distance;
    }
  } else {
    if (objectDetected) {
      // Se o objeto estava detectado e agora saiu da faixa
      if (firstDistance != -1 && lastDistance != -1) {
        if (lastDistance < firstDistance) {
          // Pessoa está entrando
          peopleEntered++;
          peopleInRoom++;
          Serial.print("Pessoa entrou! Total de pessoas na sala: ");
          Serial.println(String(peopleInRoom));
        } else if (lastDistance > firstDistance && peopleInRoom > 0) {
          // Pessoa está saindo
          peopleInRoom--;
          peopleExited++;
          Serial.print("Pessoa saiu! Total de pessoas na sala: ");
          Serial.println(String(peopleInRoom));
        }
      }
      // Reseta as variáveis
      objectDetected = false;
      firstDistance = -1;
      lastDistance = -1;
    }
  }

  // Controle do LED baseado no número de pessoas na sala
  if (peopleInRoom >= targetPeopleCount && !digitalRead(LED_PIN)) {
    digitalWrite(LED_PIN, HIGH); // Acende o LED
  } else if (peopleInRoom < targetPeopleCount && digitalRead(LED_PIN)) {
    digitalWrite(LED_PIN, LOW); // Apaga o LED
  }

   // Envia os dados formatados para o Python
  Serial.print(peopleInRoom);
  Serial.print(",");
  Serial.print(peopleEntered);
  Serial.print(",");
  Serial.println(peopleExited);
  delay(100); // Pequena pausa para estabilização
}

int getDistance() {
  // Limpa o pino TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Envia um pulso de 10 microsegundos para o pino TRIG
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Lê o tempo do pino ECHO
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calcula a distância
  return duration / (2 * 29.1);
}
