## UNIVERSIDADE FEDERAL DO RIO GRANDE DO NORTE

## ESCOLA DE CIÊNCIAS E TECNOLOGIAS

## FUNDAMENTOS DE CIRCUITOS E SISTEMAS CONTROLADOS

### ANÁLISE ESTATÍSTICA E CONTROLE DE AR CONDICIONADO COM SENSOR DE PRESENÇA

**Discentes:**
Pedro Arthur Oliveira dos Santos

Jerfeson da Silva Santos

Maria Beatryz Salviano de Silveira

Juan Pablo Hortêncio Ferreira

Marcelo Jorge Oliveira de Albuquerque Melo

**Orientador:**
Prof. Alexandre Guimaraes

Natal/RN
2024

### RESUMO

Neste projeto desenvolvemos um sensor de presença que pode acionar e controlar a intensidade de dispositivos eletrônicos, como luzes e ar-condicionado, com base na quantidade de pessoas presentes em uma sala de aula. O sistema detecta a entrada e saída de pessoas e conta o número de pessoas na sala em tempo real usando um sensor ultrassônico HC-SR04 acoplado a um Arduino. O projeto coleta dados para análise estatística e ativa automaticamente os dispositivos. Isso permite a criação de gráficos que mostram padrões de presença, como horários de pico de entrada e saída dos alunos. A validação prática mostrou que o sistema funciona bem, confirmando que a automação e o monitoramento são feitos de forma precisa e confiável.

**Palavras-chave:** sensor de presença, arduino, controle automatizado, monitoramento em tempo real, análise estatística, HC-SR04, automação de dispositivos.

### SUMÁRIO

1.  [INTRODUÇÃO](#introducao)
2.  [FUNDAMENTAÇÃO TEÓRICA](#fundamentacao-teorica)
    2.1 [Funcionamento do sensor HC-SR04](#funcionamento-do-sensor-hc-sr04)
    2.2 [Funcionamento do projeto e diagrama de blocos](#funcionamento-do-projeto-e-diagrama-de-blocos)
3.  [DESENVOLVIMENTO](#desenvolvimento)
    3.1 [Implementação da lógica e código do arduino no Arduino IDE](#implementacao-da-logica-e-codigo-do-arduino-no-arduino-ide)
    3.2 [Implementação da análise gráfica usando python](#implementacao-da-analise-grafica-usando-python)
    3.3 [Simulação no TinkerCad](#simulacao-no-tinkercad)
4.  [RESULTADOS E DISCUSSÃO](#resultados-e-discussao)
5.  [CONCLUSÃO](#conclusao)
6.  [REFERÊNCIAS](#referencias)

### 1\. INTRODUÇÃO <a name="introducao"></a>

A ideia do presente projeto que será apresentado neste relatório é um sensor de presença capaz de controlar o acionamento/desligamento de por exemplo um ar condicionado ou uma luz, dependendo da quantidade de pessoas que estão em uma sala de aula, por exemplo. O projeto se baseia em um sensor ultrasônico acoplado com um arduino conectado à uma fonte de tensão de 5 volts que irá acionar um pino do arduino para acionar o dispositivo desejado.
Além disso o sensor também monitora a quantidade de pessoas que entram, saem e que estão atualmente na sala, com isso também é possível fazer uma análise estatística por meio de gráficos para conseguir visualizar os horários de pico que os alunos chegam para a aula, além do horário que os alunos costumam sair, com isso pode ser analisado e implementada uma metodologia mais eficaz na sala da aula que seja produtiva em relação ao tempo médio que os alunos conseguem ficar focados numa aula, por exemplo.

### 2\. FUNDAMENTAÇÃO TEÓRICA <a name="fundamentacao-teorica"></a>

#### 2.1 FUNCIONAMENTO DO SENSOR HC-SR04 <a name="funcionamento-do-sensor-hc-sr04"></a>

Um dos blocos mais importantes do projeto é o sensor HC-SR04, que é um sensor ultrasônico que emite periodicamente pulsos sonoros de alta frequência pelo pino “echo”, e capta a saída pelo pino ativador “trigger” que fica conectado à uma saída digital do arduino, dessa maneira é possível medir as distâncias que um objeto fica a partir do centro do sensor, que tem uma área de cobertura cônica, como mostra a figura abaixo.

Figura 1 - Sensor HC-SR04 no tinkercad

Assim então é calculado o tempo em que o pino "Echo" ficou em nível alto, ou seja, recebeu um sinal emitido pelo trigger e refletido pelo objeto que estava na frente do sensor.

#### 2.2 FUNCIONAMENTO DO PROJETO E DIAGRAMA DE BLOCOS <a name="funcionamento-do-projeto-e-diagrama-de-blocos"></a>

Os componentes usados neste projeto são:

  * Arduino Uno R3
  * Sensor Ultrasônico HC-SR04
  * Jumpers
  * Mini ProtoBoard
  * Led Verde
  * 1 Resistor
  * Display ou Computador para visualização gráfica

A figura abaixo mostra o diagrama de blocos do sensor de presença.

Figura 2 - Diagrama de blocos do projeto

O diagrama de blocos mostra a característica dual do sistema do projeto implementado, a saída do arduino pode ser vista como uma soma, ou um arranjo paralelo de duas saídas, que são exatamente as duas aplicações propostas para o projeto, e são saídas de natureza diferentes, logo não é possível achar uma função de transferência para o sistema.
Para implementar o sensor de presença para a aplicação deste projeto, foi necessário adaptar o uso deste sensor para ele ser capaz de medir duas distâncias, a fim de verificar a direção que uma pessoa está se movendo, para dizer se ela está saindo ou entrando na sala, e assim efetuar a contagem de entradas ou saídas, além da contagem de quantas pessoas estão atualmente na sala.
E a partir da contagem de pessoas dentro da sala, o projeto visa automatizar o acionamento de algum dispositivo eletrônico, como por exemplo um ar condicionado.
Além disso, é possível também fazer uma análise estatística temporal posterior para verificar padrões, horários que os alunos saem mais da aula, horário que mais chegam, a fim de analisar a produtividade e atenção dos alunos em um horário de aula, o software que será usado para fazer os gráficos será o Python, e irá plotar os gráficos das entradas, saídas e pessoas na sala em função do tempo.

### 3\. DESENVOLVIMENTO <a name="desenvolvimento"></a>

#### 3.1 IMPLEMENTAÇÃO DA LÓGICA E CÓDIGO DO ARDUINO NO ARDUINO IDE <a name="implementacao-da-logica-e-codigo-do-arduino-no-arduino-ide"></a>

A lógica do código do arduino que foi utilizado serve para medir as distâncias de cada pessoa, e verificar se ela está se afastando ou se distanciando, para diferenciar se é uma entrada ou uma saída.
A função `getDistance()` é um dos blocos principais na implementação do sensor, e é usada em um loop condicional para verificar uma distância inicial e uma final, para então decidir se alguém saiu ou entrou.

A parte principal do código que faz o sensor ultrassônico funcionar como um sensor de presença é a parte dentro do `loop()` principal, que implementa a lógica condicional de verificação de entradas e saídas. O código implementado no arduino é mostrado abaixo.

```arduino
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
const int thresholdDistance = 100; // Ajuste conforme necessário para a sua instalação
const int targetPeopleCount = 1;
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
          //Serial.print("Pessoa entrou! Total de pessoas na sala:" + String(peopleEntered));
          //Serial.println("Pessoas na sala: " + String(peopleInRoom));
        } else if (lastDistance > firstDistance && peopleInRoom > 0) {
          // Pessoa está saindo
          peopleInRoom--;
          peopleExited++;
          //Serial.println("Saídas: " + String(peopleExited));
          //Serial.println("Pessoas na sala: " + String(peopleInRoom));
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

  // Envia os dados para o Serial Plotter

  Serial.print(peopleEntered);
  Serial.print(" ");  // Espaço separa os valores
  Serial.print(peopleExited);
  Serial.print(" ");
  Serial.println(peopleInRoom);  // O Serial Plotter usa println para plotar uma nova linha

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
```

#### 3.2 IMPLEMENTAÇÃO DA ANÁLISE GRÁFICA USANDO PYTHON <a name="implementacao-da-analise-grafica-usando-python"></a>

Foi utilizado python para ler a porta serial do computador onde o arduino estiver conectado usando a biblioteca pyserial, para começar a execução em tempo real com o arduíno.
O código em python tem o objetivo de armazenar os dados das entradas, saídas e pessoas na sala, e também o tempo sincronizado com o relógio utilizando o objeto datetime da biblioteca time e análise dos gráficos posteriormente.
Foram utilizadas além dessas, as bibliotecas matplotlib, numpy e scipy para fazer as operações matemáticas e a plotagem dos gráficos.
O código em python é mostrado abaixo

```python
import serial
import csv
import time
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
# Configura a porta serial (substitua 'COM3' pela porta correta do seu Arduino)
ser = serial.Serial('COM3', 9600)
# Arrays para armazenar os dados
timestamps = []
entradas = []
saidas = []
pessoas_na_sala = []
# Define o nome do arquivo CSV
filename = "contagem_pessoas.csv"

# Captura o instante atual como hora de início
hora_inicio = datetime.now()
# Hora alvo para encerrar a coleta (14:15)
hora_alvo = datetime.strptime("08:43:00", "%H:%M:%S").time()
# Abre o arquivo CSV para escrita
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "Entradas", "Saidas", "Pessoas na Sala"])
    coleta_ativa = True  # Variável de controle do loop
    while coleta_ativa:
        try:
            # Lê a linha da serial
            line = ser.readline().decode('utf-8').strip()
            print(f"Recebido: '{line}'")  # Depuração
            # Divide os dados recebidos usando espaço como delimitador
            if line:
                data = line.split()
                print(f"Dados: {data}")  # Depuração
                if len(data) == 3:
                    try:
                        current_time = datetime.now()
                        timestamps.append(current_time)
                        entradas.append(int(data[0]))
                        saidas.append(int(data[1]))
                        pessoas_na_sala.append(int(data[2]))
                        #Escreve os dados no arquivo CSV com timestamp
                        writer.writerow([current_time.strftime("%Y-%m-%d %H:%M:%S")] + data)
                    except ValueError:
                        print("Erro ao converter dados para inteiro.")
                else:
                    print(f"Formato de dados inesperado: {data}")
            # Verifica se a hora atual é maior ou igual à hora alvo
            if datetime.now().time() >= hora_alvo:
                print("Hora alvo atingida. Encerrando coleta.")
                coleta_ativa = False

        except KeyboardInterrupt:
            print("Coleta de dados interrompida manualmente.")
            coleta_ativa = False
ser.close()
intervalo_inicio = (hora_inicio.hour * 3600 + hora_inicio.minute * 60 + hora_inicio.second) / 3600
intervalo_fim = (hora_alvo.hour * 3600 + hora_alvo.minute * 60 + hora_alvo.second) / 3600
# Verifica se listas não estão vazias antes de plotar
if timestamps and entradas and saidas and pessoas_na_sala:
    # Converte listas para arrays numpy
    timestamps = np.array([(t.hour * 3600 + t.minute * 60 + t.second) / 3600 for t in timestamps])
    entradas = np.array(entradas)
    saidas = np.array(saidas)
    pessoas_na_sala = np.array(pessoas_na_sala)
    # Plotagem dos dados
    plt.figure(figsize=(12, 8))
    plt.plot(timestamps, entradas, label='Entradas')
    plt.plot(timestamps, saidas, label='Saídas')
    plt.plot(timestamps, pessoas_na_sala, label='Pessoas na Sala')
    plt.xlabel('Hora do Dia (em Horas)')
    plt.ylabel('Contagem')
    plt.title('Contagem de Entradas, Saídas e Pessoas na Sala ao Longo do Dia')
    #x_ticks = np.arange(intervalo_inicio, intervalo_fim + 1, 1)
    #x_labels = [f'{int(i):02}:00' for i in x_ticks]
    x_ticks = np.linspace(intervalo_inicio, intervalo_fim, num=10)  # Define 10 ticks igualmente espaçados
    x_labels = [f'{int(t)}:{int((t - int(t)) * 60):02}' for t in x_ticks]  # Converte ticks para formato HH:MM
    plt.xticks(x_ticks, x_labels)
    plt.xlim(intervalo_inicio, intervalo_fim)
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()
    # Realiza a Transformada de Fourier para Entradas, Saídas e Pessoas na Sala
    def plot_fft(data, ax, title):
        N = len(data)
        T = (timestamps[-1] - timestamps[0]) / (N - 1) if N > 1 else 1  # Período entre amostras
        yf = fft(data)
        xf = fftfreq(N, T)[:N // 2]
        ax.plot(xf,(2/N)*np.abs(yf[:N // 2]),"cyan",label = "Amplitude")
        ax.plot(xf,(2/N)*np.angle(yf[:N // 2]),"purple",label = "Fase")
        ax.set_title(title)
        ax.set_xlabel('Frequência (Hz)')
        ax.legend()
        ax.set_xlim(0, 2000)
    plot_fft(entradas, plt.gca(), 'Transformada de Fourier do Gráfico das Entradas')
    plt.show()
    plot_fft(saidas, plt.gca(), 'Transformada de Fourier do Gráfico das Saídas')
    plt.show()
    plot_fft(pessoas_na_sala, plt.gca(), 'Transformada de Fourier do Gráfico

```

### 3.3 SIMULAÇÃO NO TINKERCAD <a name="simulacao-no-tinkercad"></a>

Foi realizada uma simulação iterativa no tinkercad, usando a iteração com o sensor HC-SR04, e foram testados os movimentos de entrada e saída para confirmar o funcionamento, mostrada no link abaixo.

https://www.tinkercad.com/things/9N51FmTgMDg-projeto-sensor-de-presenca-fcsc


### 4. RESULTADOS E DISCUSSÃO <a name="resultados-e-discussao"></a>

O projeto foi concluído com sucesso. A quantidade de pessoas que entraram e saíram da sala foi calculada corretamente. Essa contagem foi utilizada para acender um LED, mostrando como o sistema funciona. Por exemplo, quando dez pessoas entravam na sala, o LED era acionado. Além disso, a análise dos gráficos permitiu que a equação que mostra a relação entre o número de entradas, saídas e o número total de pessoas na sala fosse verificada: $N_{entradas} - N_{saídas} = N_{pessoas\_na\_sala}$.

Figura 04 - Implementação prática

Figura 05 - Verificação do LED

### 5. CONCLUSÃO <a name="conclusao"></a>

A análise dos gráficos no domínio temporal e na frequência revelou padrões periódicos nas entradas e saídas das pessoas da sala. Esses resultados confirmam a eficácia do sistema implementado, evidenciando que o monitoramento em tempo real de pessoas em um ambiente pode ser realizado de maneira precisa e confiável, com a possibilidade de usar sinais para automatizar o acionamento de um dispositivo.
