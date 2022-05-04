"""
 Script da cena deve conter:
 
 simRemoteApi.start(1000, 1300, false, true) 
 Para ativar modo síncrono, usando porta 1000 ou outra que funcione
 
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Móvel - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Data: 14/05/2019
"""

import vrep
import time
import sys
import numpy as np

#Fecha todas as comunicações existentes
vrep.simxFinish(-1) 

#Inicializa RemoteAPI com parâmetros (IP,PORT,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs)
clientID = vrep.simxStart('127.0.0.1',1000,True,True,2000,5) 

#Ativa modo síncrono da RemoteAPI
vrep.simxSynchronous(clientID, True) 

if clientID != -1:
    print ('Servidor conectado!')
else:
    #Falha de conexão
    print ('Problemas para conectar o servidor!')
    sys.exit('Could not connect')
   
# Handles da Cena [Todos os Objetos que compõem o robô]
_, Handle_Corpo = vrep.simxGetObjectHandle(clientID,'Corpo',vrep.simx_opmode_oneshot_wait)
_, Handle_Alvo = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)

#Variáveis  de Simulação (Parâmetros medidos/estabelecidos da cena do V-REP)
t = 0       # Guarda tempo de simulação no Python
dt = 0.05   # Intervalo de integração
x = 0       # Posição X inicial do corpo
y = 0       # Posição Y inicial do corpo

#Reset da posição inicial do robô para origem das coordenadas
vrep.simxSetObjectPosition(clientID, Handle_Corpo, -1, [x,y,0], vrep.simx_opmode_oneshot_wait)
vrep.simxSetObjectPosition(clientID, Handle_Alvo, -1, [x+2,y+2,0], vrep.simx_opmode_oneshot_wait)

#Função que simula algoritmos globais de sensoriamento [Ex. Visão Computacional]
def Obter_Posicao(handle): 
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1,vrep.simx_opmode_streaming)
    _, ori_body = vrep.simxGetObjectOrientation(clientID, handle,-1,vrep.simx_opmode_streaming)

    x = pos[0]   
    y = pos[1]
    
    return x, y

#Função que simula algoritmos globais de sensoriamento [Ex. Visão Computacional]
def Setar_Posicao(handle, x, y): 
    vrep.simxSetObjectPosition(clientID, handle, -1, [x, y, 0], vrep.simx_opmode_streaming)

def Simular_Sistema(x_ant, y_ant, ux, uy): 
    xp = ux
    yp = uy
    
    x = x_ant + xp * dt
    y = y_ant + yp * dt
    
    return x,y

Kp = 5
Ki = 0.5
Kd = 0.05;

I_x = 0
I_y = 0

erro_x = 0
erro_y = 0

erro_x_a = 0
erro_y_a = 0

#Loop de controle do robô
while vrep.simxGetConnectionId(clientID) != -1:
    t0 = time.perf_counter()
    
    t += dt
    
    xd, yd = Obter_Posicao(Handle_Alvo)
    
    erro_x = xd - x
    erro_y = yd - y
    
    P_x = Kp*erro_x
    I_x = I_x + erro_x*dt
    D_x = (erro_x - erro_x_a)/dt
    
    P_y = Kp*erro_y
    I_y = I_y + erro_y*dt
    D_y = (erro_y - erro_y_a)/dt
    
    erro_x_a = erro_x
    erro_y_a = erro_y
    
    ux = P_x*Kp + I_x*Ki + D_x*Kd
    uy = P_y*Kp + I_y*Ki + D_y*Kd
    
    ### Saturação 
    umax = 10
    
    if(np.abs(ux) > umax):
        ux = np.sign(ux)*umax
    if(np.abs(uy) > umax):
        uy = np.sign(uy)*umax
    ### Saturação
    
    x, y = Simular_Sistema(x, y, ux, uy)
      
    Setar_Posicao(Handle_Corpo, x, y)
    
    #Print de dados
    print("xd: %1.2f  yd: %1.2f" % (xd, yd))
    print("x : %1.2f  y : %1.2f" % (x, y))
    print("erro_x : %1.2f  erro_y : %1.2f" % (erro_x, erro_y))
    #print("Tempo de Simulação: %1.2f [seg]" % (t))
    
    vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
    
    while(time.perf_counter()-t0 <= dt): _ # Para o loop até 50ms
    
#Desligamento
vrep.simxFinish(clientID) # fechando conexao com o servidor
print ('Conexao fechada!')