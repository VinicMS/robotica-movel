"""
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Móvel - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Revisado em: 22/05/2022
 
 # Modificação dos exemplos para controle direto da IDE Python
 # Utilize o PLAY e o CTRL+C do Spyder IDE para controlar a simulação sem 
   necessidade de clicar nos botões do CoppeliaSim
"""

import vrep
import time
import sys
import numpy as np

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
   print ('Servidor conectado!') 
else:
    print ('Problemas para conectar o servidor!')
    sys.exit()

#Ativa modo síncrono da RemoteAPI
vrep.simxSynchronous(clientID, True) 

#Inicia a simulação
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

# Handles da Cena [Todos os Objetos que compõem o robô]
_, TargetbodyHandle = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)
_, leftMotorHandle = vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_oneshot_wait)
_, rightMotorHandle = vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_oneshot_wait)
_, bodyHandle = vrep.simxGetObjectHandle(clientID,'Robo_Ifes',vrep.simx_opmode_oneshot_wait)

#Chama função genérica de reset do encoder. Está no Child Script das rodas no simulador V-REP
vrep.simxCallScriptFunction(clientID,'RightWheel',vrep.sim_scripttype_childscript,'Reset_Right_Encoder',[],[],[],bytearray(),vrep.simx_opmode_blocking)
vrep.simxCallScriptFunction(clientID,'LeftWheel',vrep.sim_scripttype_childscript,'Reset_Left_Encoder',[],[],[],bytearray(),vrep.simx_opmode_blocking)

R = 0.033   # Raio da Roda
L = 0.31    # Distância entre-rodas
    
#Função de Controle do Uniciclo a partir de (v,w)
def ControleUniciclo(u,w):
    wd=(2*u+w*L)/(2*R)
    we=(2*u-w*L)/(2*R)
    
    vrep.simxSetJointTargetVelocity(clientID,  leftMotorHandle, we, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, wd, vrep.simx_opmode_streaming)

#Função que simula algoritmos globais de sensoriamento [Ex. Visão Computacional]
def Obter_Posicao(handle): 
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1,vrep.simx_opmode_streaming)
    _, ori_body = vrep.simxGetObjectOrientation(clientID, handle,-1,vrep.simx_opmode_streaming)

    phi = ori_body[2] 
    phi = np.arctan2(np.sin(phi), np.cos(phi)) #[rad] - Correção domínio [-pi, pi]
    x = pos[0]   
    y = pos[1]
    
    return x, y, phi

#Função que captura pulsos de encoders simulados nas rodas do robô
#Encoder está simulado no Child Script de cada roda na cena do V-REP 
def getEncoderPulses():
    _, pe = vrep.simxGetIntegerSignal(clientID,'Left_Encoder',vrep.simx_opmode_streaming)
    _, pd = vrep.simxGetIntegerSignal(clientID,'Right_Encoder',vrep.simx_opmode_streaming)
     
    return pe, pd
  
def norm_ang(a):
    return np.arctan2(np.sin(a),np.cos(a))

def main():    
    #Variáveis 
    t = 0       # Guarda tempo de simulação no Python
    dt = 0.05   # Intervalo de integração
    xo = 0      # Posição da Odometria [xo,yo,phio]
    yo = 0      # Posição da Odometria [xo,yo,phio]
    phio = 0    # Posição da Odometria [xo,yo,phio]
    N = 20      # Resolução do Encoder
    pd_ant = 0  # Auxiliar - Cáluculo da variação de pulsos
    pe_ant = 0  # Auxiliar - Cáluculo da variação de pulsos
    Qe = 0      # Velocidade [pulsos/seg]
    Qd = 0      # Velocidade [pulsos/seg]
    k = 0       #auxiliar
    pe = 0
    pd = 0
    dpe = 0
    dpd = 0
    
    #Loop de controle do robô
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter()
        
        t += dt 
        
        #Obtém pulsos do encoder (pl- pulsos da esquerda) (pr - pulsos da direita)
        pe, pd = getEncoderPulses() 
        
        dpe = pe - pe_ant
        dpd = pd - pd_ant
    
        pe_ant, pd_ant = pe, pd
        
        #Calcula posição por odometria
        xo = 0
        yo = 0
        phio = 0
        
        #Controle de posição   
        xg, yg, phig = Obter_Posicao(TargetbodyHandle)   
         
        u = 0.1
        w = 0.0
        
        ControleUniciclo(u,w) #Movimento de exemplo
        
        # Teste da odometria contra dados do simulador)
        x_real, y_real, phi_real = Obter_Posicao(bodyHandle)
        
        #print("Pose Real        (x,y,phi): (%1.2f, %1.2f, %1.2f)" % (x_real, y_real, phi_real))
        #print("Pose Odometria   (x,y,phi): (%1.2f, %1.2f, %1.2f)" % (xo, yo, phio))
        #print("Pose do Alvo     (x,y,phi): (%1.2f, %1.2f, %1.2f)" % (xg, yg, phig))
        print("Pulsos dos Encoders (E, D): (%d, %d)" % (pe,pd))
        #print("we, wd: (%d, %d)" % (wl,wr))
        
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        
        while(time.perf_counter()-t0 <= dt): _ # Para o loop até 50ms
    

try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)