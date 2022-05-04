"""
 Exercício PID:
 - Abrir a cena no simulador V-REP e iniciar simulação
 - Iniciar script na IDE Spyder (Botão PLAY acima e STOP no console ao lado >>)
 - Alterar o código para obter um PID que realiza o controle de posição do veículo simulado
 - Considerar dt = 50ms
 - Restante do código inspirado em exemplos da RemoteAPI para linguagem Python do simulador V-REP
 - Documentação acessível em: http://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm
 - Child Script da cena deve conter incialização da API Remota em modo Síncrono simRemoteApi.start(1000, 1300, false, true) dentro da função sysCall_init()
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
    sys.exit('Programa finalizado!')
    
# Handle do Robô (Motor e Corpo)
_, MotorHandle = vrep.simxGetObjectHandle(clientID,'Motor',vrep.simx_opmode_oneshot_wait)
_, BodyHandle = vrep.simxGetObjectHandle(clientID,'Carro',vrep.simx_opmode_oneshot_wait)

# Handle do Alvo
_, TargetBodyHandle = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)
        
#Função para obter posição
def getPosition(handle):
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1, vrep.simx_opmode_streaming)
    return pos[1]

#Função para comandar motor
def setSpeed(u): #Seta velocidade angular das rodas em [rad/s]
    vrep.simxSetJointTargetVelocity(clientID,  MotorHandle, u, vrep.simx_opmode_streaming)

Kp = 3.0
Ki = 0.1
Kd = 0.05;

dt=0.05

I_x = 0

erro_x = 0

erro_x_a = 0

#Loop de controle do robô
while vrep.simxGetConnectionId(clientID) != -1:
    t0 = time.perf_counter() #Controle de tempo
    
    xd = getPosition(TargetBodyHandle) #Captura posição desejada
     
    x = getPosition(BodyHandle) #Captura posição do veículo
    
    #####################Controlador PID#####################
    erro_x = xd - x
    
    P_x = Kp*erro_x
    I_x = I_x + erro_x*dt
    D_x = (erro_x - erro_x_a)/dt
    
    erro_x_a = erro_x
    
    #if(ux > 1)
    
    ux = P_x*Kp + I_x*Ki + D_x*Kd
    
    
    print(erro_x)
        
    
    setSpeed(ux) # Exemplo de controle em malha aberta. Rode a simulação e veja o que acontece
    #####################Controlador PID#####################
    
    print("x_d: %1.2f  x: %1.2f erro: %1.2f  u: %1.2f" % (xd, x, erro_x, ux))

    vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
    
    while(time.perf_counter()-t0 <= 0.05): _ # Pára o loop até 50ms
    
#Desligamento
vrep.simxFinish(clientID) # fechando conexao com o servidor
print ('Conexao fechada!')