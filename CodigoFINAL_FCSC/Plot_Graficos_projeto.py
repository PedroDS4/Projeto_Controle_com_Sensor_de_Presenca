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

                        # Escreve os dados no arquivo CSV com timestamp
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

    plot_fft(pessoas_na_sala, plt.gca(), 'Transformada de Fourier do Gráfico de Pessoas na Sala')
    plt.show()

    plt.tight_layout()
    plt.grid(True)


else:
    print("As listas estão vazias. Verifique a coleta de dados.")
